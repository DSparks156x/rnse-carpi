//! The virtual CAN bus carpid connects to (`carpi_can::sim`). Each carpid
//! port is one TCP connection: `HELLO rx <iface>` gets that interface's
//! frames, `HELLO tx <iface>` sends. As on SocketCAN, carpid's receiving
//! socket also sees what its sending socket put on the bus.

use crate::Sim;
use carpi_can::Frame;
use carpi_can::sim::{line, parse};
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpListener;
use tokio::sync::mpsc;

struct Client {
    id: u64,
    iface: String,
    out: mpsc::UnboundedSender<String>,
}

#[derive(Default)]
pub struct HubStatus {
    pub addr: String,
    pub rx: usize,
    pub tx: usize,
}

pub struct Hub {
    listener: Mutex<Option<TcpListener>>,
    clients: Mutex<Vec<Client>>,
    status: Arc<Mutex<HubStatus>>,
}

impl Hub {
    pub async fn bind(addr: &str) -> anyhow::Result<Arc<Hub>> {
        let listener = TcpListener::bind(addr).await.map_err(|e| {
            anyhow::anyhow!("virtual CAN hub on {addr}: {e} (is another rnse-sim running?)")
        })?;
        let status = HubStatus {
            addr: listener.local_addr()?.to_string(),
            ..Default::default()
        };
        Ok(Arc::new(Hub {
            listener: Mutex::new(Some(listener)),
            clients: Mutex::new(Vec::new()),
            status: Arc::new(Mutex::new(status)),
        }))
    }

    pub fn status(&self) -> Arc<Mutex<HubStatus>> {
        self.status.clone()
    }

    /// Sends `frame` on `iface` to every receiving connection but `except`.
    pub fn broadcast(&self, iface: &str, frame: &Frame, except: Option<u64>) {
        let l = line(iface, frame);
        let mut clients = self.clients.lock().unwrap();
        clients.retain(|c| {
            if c.iface != iface || Some(c.id) == except {
                return true;
            }
            c.out.send(l.clone()).is_ok()
        });
    }

    fn count(&self) {
        let rx = self.clients.lock().unwrap().len();
        self.status.lock().unwrap().rx = rx;
    }

    pub async fn run(self: Arc<Self>, sim: Arc<Mutex<Sim>>) {
        let listener = self.listener.lock().unwrap().take().expect("run once");
        let mut next = 1u64;
        loop {
            let Ok((sock, _)) = listener.accept().await else {
                continue;
            };
            let _ = sock.set_nodelay(true);
            let id = next;
            next += 1;
            let hub = self.clone();
            let sim = sim.clone();
            tokio::spawn(async move {
                let (rd, mut wr) = sock.into_split();
                let mut lines = BufReader::new(rd).lines();
                let Ok(Some(hello)) = lines.next_line().await else {
                    return;
                };
                let mut parts = hello.split_whitespace();
                let (Some("HELLO"), Some(role), Some(iface)) =
                    (parts.next(), parts.next(), parts.next())
                else {
                    return;
                };
                let iface = iface.to_string();
                match role {
                    "rx" => {
                        let (out, mut rx) = mpsc::unbounded_channel::<String>();
                        hub.clients.lock().unwrap().push(Client { id, iface, out });
                        hub.count();
                        while let Some(l) = rx.recv().await {
                            if wr.write_all(l.as_bytes()).await.is_err() {
                                break;
                            }
                        }
                        hub.clients.lock().unwrap().retain(|c| c.id != id);
                        hub.count();
                    }
                    "tx" => {
                        hub.status.lock().unwrap().tx += 1;
                        while let Ok(Some(l)) = lines.next_line().await {
                            let Some((iface, frame)) = parse(&l) else {
                                continue;
                            };
                            // Loop back to carpid's own receiver, then to the car.
                            hub.broadcast(&iface, &frame, None);
                            sim.lock().unwrap().from_carpid(Instant::now(), &frame);
                        }
                        hub.status.lock().unwrap().tx -= 1;
                    }
                    _ => {}
                }
            });
        }
    }
}
