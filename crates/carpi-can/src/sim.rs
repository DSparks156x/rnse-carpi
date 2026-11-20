//! Virtual CAN over TCP, for the desktop simulator (`tools/rnse-sim`).
//!
//! With `CARPI_SIM_CAN=host:port` set, [`crate::Port::open`] and
//! [`crate::TxPort::open`] connect to a hub there instead of opening
//! SocketCAN, on any OS. The hub is the simulator: it plays the rest of the
//! car (RNS-E, cluster, gateway) and relays frames between its nodes.
//!
//! Wire: text lines. The client opens with `HELLO rx <iface>` (receive the
//! frames on `<iface>`) or `HELLO tx <iface>` (send only), then frames as
//! candump log lines without a timestamp: `ican 461#373001002000`. Like
//! SocketCAN, a receiving connection also gets the frames our own sending
//! connection put on the bus.

use crate::{Frame, RxFrame, candump};
use std::io;
use std::time::{Instant, SystemTime};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader, Lines};
use tokio::net::TcpStream;
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::sync::Mutex;

/// Environment variable naming the hub.
pub const ENV: &str = "CARPI_SIM_CAN";

/// The hub address, when the simulator is in use.
pub fn hub() -> Option<String> {
    std::env::var(ENV).ok().filter(|s| !s.trim().is_empty())
}

/// One frame as a wire line, newline included.
pub fn line(iface: &str, frame: &Frame) -> String {
    let id = if frame.extended {
        format!("{:08X}", frame.id)
    } else {
        format!("{:03X}", frame.id)
    };
    let data: String = frame.data().iter().map(|b| format!("{b:02X}")).collect();
    format!("{iface} {id}#{data}\n")
}

/// A wire line back to (iface, frame). `None` for anything else.
pub fn parse(line: &str) -> Option<(String, Frame)> {
    candump::parse_line(line).map(|l| (l.iface, l.frame))
}

fn connect(addr: &str, role: &str, iface: &str) -> io::Result<TcpStream> {
    use std::io::Write;
    let mut s = std::net::TcpStream::connect(addr)?;
    s.set_nodelay(true)?;
    // Blocking, before tokio takes it: the hello is a few bytes.
    s.write_all(
        format!(
            "HELLO {role} {iface}
"
        )
        .as_bytes(),
    )?;
    s.set_nonblocking(true)?;
    TcpStream::from_std(s)
}

/// Receiving side.
pub struct SimPort {
    iface: String,
    lines: Lines<BufReader<OwnedReadHalf>>,
    _wr: OwnedWriteHalf,
}

impl SimPort {
    pub fn open(addr: &str, iface: &str) -> io::Result<Self> {
        let (rd, wr) = connect(addr, "rx", iface)?.into_split();
        Ok(SimPort {
            iface: iface.to_string(),
            lines: BufReader::new(rd).lines(),
            _wr: wr,
        })
    }

    /// Next frame on our interface; `Ok(None)` when the hub went away.
    pub async fn recv(&mut self) -> io::Result<Option<RxFrame>> {
        while let Some(l) = self.lines.next_line().await? {
            if let Some((iface, frame)) = parse(&l)
                && iface == self.iface
            {
                return Ok(Some(RxFrame {
                    frame,
                    at: Instant::now(),
                    wall: SystemTime::now(),
                }));
            }
        }
        Ok(None)
    }
}

/// Sending side.
pub struct SimTx {
    iface: String,
    wr: Mutex<OwnedWriteHalf>,
    _rd: OwnedReadHalf,
}

impl SimTx {
    pub fn open(addr: &str, iface: &str) -> io::Result<Self> {
        let (rd, wr) = connect(addr, "tx", iface)?.into_split();
        Ok(SimTx {
            iface: iface.to_string(),
            wr: Mutex::new(wr),
            _rd: rd,
        })
    }

    pub async fn send(&self, frame: &Frame) -> io::Result<()> {
        let l = line(&self.iface, frame);
        self.wr.lock().await.write_all(l.as_bytes()).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_roundtrip() {
        let f = Frame::new(0x461, &[0x37, 0x30, 0x01, 0x00, 0x20, 0x00]).unwrap();
        let l = line("ican", &f);
        assert_eq!(l, "ican 461#373001002000\n");
        assert_eq!(parse(&l), Some(("ican".to_string(), f)));
        let e = Frame::new_extended(0x18DA_F110, &[]).unwrap();
        assert_eq!(parse(&line("diag", &e)), Some(("diag".to_string(), e)));
    }

    #[tokio::test]
    async fn rx_and_tx_through_a_hub() {
        let hub = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = hub.local_addr().unwrap().to_string();
        let a2 = addr.clone();
        let rx = tokio::spawn(async move { SimPort::open(&a2, "ican").unwrap() });
        let (rx_sock, _) = hub.accept().await.unwrap();
        let mut rx = rx.await.unwrap();
        let tx = SimTx::open(&addr, "ican").unwrap();
        let (tx_sock, _) = hub.accept().await.unwrap();

        let mut tx_lines = BufReader::new(tx_sock).lines();
        assert_eq!(
            tx_lines.next_line().await.unwrap().as_deref(),
            Some("HELLO tx ican")
        );
        let f = Frame::new(0x6C0, &[0x10, 0x00, 0x15]).unwrap();
        tx.send(&f).await.unwrap();
        let got = tx_lines.next_line().await.unwrap().unwrap();

        let (r, mut w) = rx_sock.into_split();
        let mut rx_lines = BufReader::new(r).lines();
        assert_eq!(
            rx_lines.next_line().await.unwrap().as_deref(),
            Some("HELLO rx ican")
        );
        // Another interface's frame is skipped, ours comes through.
        w.write_all(b"diag 200#01\n").await.unwrap();
        w.write_all(format!("{got}\n").as_bytes()).await.unwrap();
        assert_eq!(rx.recv().await.unwrap().unwrap().frame, f);
        drop(w);
        drop(rx_lines);
        assert!(rx.recv().await.unwrap().is_none());
    }
}
