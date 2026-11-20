//! A stand-in for LIVI's carpid bridge (`livi/native/crates/livi-carpi`):
//! the same API connection LIVI makes, so carpid sees "LIVI connected" and
//! routes keys as it would in the car.
//!
//! - pushes focus, projection, nav, media and call the way LIVI does
//!   (`BridgeUpdate`), from the simulator's controls;
//! - takes carpid's `Command`s (car keys for the projection, toasts) and
//!   shows what Android Auto would get, via the same mapping as LIVI
//!   (`livi-carpi/src/input.rs` then `livi-aa-stack/src/commands.rs`);
//! - subscribes to the values LIVI reads (`livi-carpi/src/values.rs`).

use carpi_proto::{
    BridgeUpdate, CallState, Command, InputAction, MediaState, NavState, Projection, Request,
    Response,
};
use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

/// What LIVI subscribes to (`livi-carpi/src/values.rs` `PREFIXES`).
pub const LIVI_PREFIXES: [&str; 14] = [
    "vehicle.speed_kmh",
    "vehicle.odometer_km",
    "engine.rpm",
    "engine.coolant_c",
    "power.battery_v",
    "fuel.",
    "brakes.",
    "trans.selector",
    "lights.",
    "body.",
    "ambient.outside_temp_c",
    "steering.angle_deg",
    "light.night",
    "light.dim_display_pct",
];

/// carpid state the simulator shows besides LIVI's values.
const EXTRA_PREFIXES: [&str; 4] = ["dis.", "ui.", "rnse.", "ignition.kl15"];

/// What the simulator's "LIVI" pushes. Compared with the last sent copy.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Push {
    pub focus: String,
    pub projection: Option<Projection>,
    pub nav: Option<NavState>,
    pub media: Option<MediaState>,
    pub call: Option<CallState>,
}

#[derive(Default)]
pub struct LiviState {
    pub want: Push,
    pub connected: bool,
    /// Commands from carpid, newest last: (when, text).
    pub events: VecDeque<(Instant, String)>,
    /// Rotary steps AA got, for the rate readout: (when, delta).
    pub rotary: VecDeque<(Instant, i64)>,
    /// Subscribed values, by name.
    pub values: BTreeMap<String, String>,
    pub last_error: Option<String>,
}

impl LiviState {
    fn event(&mut self, text: String) {
        self.events.push_back((Instant::now(), text));
        while self.events.len() > 60 {
            self.events.pop_front();
        }
    }
}

/// What Android Auto receives for one action, as LIVI sends it: LIVI's
/// `input::aa` then `commands::command_action`. Mirrored by hand; keep in
/// step with those two files.
pub fn aa_keys(action: &InputAction) -> Vec<String> {
    use InputAction as A;
    let key = |name: &str, down: bool| format!("{name} {}", if down { "down" } else { "up" });
    let click = |name: &str| format!("{name} click");
    match action {
        A::Up => vec![click("DPAD_UP")],
        A::Down => vec![click("DPAD_DOWN")],
        A::Left => vec![key("DPAD_LEFT", true), key("DPAD_LEFT", false)],
        A::Right => vec![key("DPAD_RIGHT", true), key("DPAD_RIGHT", false)],
        // One Rotary(±1) per step, at most 16 (`input::MAX_STEPS`).
        A::Rotate { steps } => {
            let n = steps.unsigned_abs().min(16) as usize;
            let d = if *steps > 0 { "+1" } else { "-1" };
            vec![format!("ROTARY {d}"); n]
        }
        A::Select { down } => vec![key("DPAD_CENTER", *down)],
        A::Back => vec![key("BACK", true), key("BACK", false)],
        A::Home => vec![click("HOME")],
        A::PlayPause => vec![click("MEDIA_PLAY_PAUSE")],
        A::Next => vec![click("MEDIA_NEXT")],
        A::Prev => vec![click("MEDIA_PREVIOUS")],
        A::CallAccept => vec![click("CALL")],
        A::CallEnd => vec![click("ENDCALL")],
        A::Voice { down } => vec![key("SEARCH", *down)],
        A::OpenNav => vec![click("NAVIGATION")],
        A::OpenPhone => vec![click("TEL")],
        A::OpenMedia => vec![click("MEDIA")],
        A::OpenPage { page } => vec![format!("(LIVI page {page}, not AA)")],
    }
}

fn describe(action: &InputAction) -> String {
    match action {
        InputAction::Rotate { steps } => format!("rotate {steps:+}"),
        InputAction::Select { down } => format!("select {}", if *down { "down" } else { "up" }),
        InputAction::Voice { down } => format!("voice {}", if *down { "down" } else { "up" }),
        InputAction::OpenPage { page } => format!("open page {page}"),
        other => format!("{other:?}").to_lowercase(),
    }
}

fn value_text(v: &Option<carpi_proto::Value>) -> String {
    match v {
        None => "—".into(),
        Some(carpi_proto::Value::Bool(b)) => b.to_string(),
        Some(carpi_proto::Value::Num(n)) => {
            if n.fract() == 0.0 {
                format!("{n:.0}")
            } else {
                format!("{n:.2}")
            }
        }
        Some(carpi_proto::Value::Text(t)) => t.clone(),
    }
}

async fn send(w: &mut tokio::net::tcp::OwnedWriteHalf, req: &Request) -> std::io::Result<()> {
    w.write_all(&carpi_proto::encode(req)).await
}

async fn read(r: &mut tokio::net::tcp::OwnedReadHalf) -> std::io::Result<Response> {
    let mut prefix = [0u8; 4];
    r.read_exact(&mut prefix).await?;
    let n = carpi_proto::body_len(prefix).map_err(std::io::Error::other)?;
    let mut body = vec![0u8; n];
    r.read_exact(&mut body).await?;
    carpi_proto::decode(&body).map_err(std::io::Error::other)
}

/// Connects to carpid at `addr` and keeps the bridge up, reconnecting.
pub async fn run(addr: String, state: Arc<Mutex<LiviState>>) {
    loop {
        match session(&addr, &state).await {
            Ok(()) => {}
            Err(e) => {
                let mut s = state.lock().unwrap();
                if s.connected || s.last_error.is_none() {
                    s.event(format!("bridge: {e}"));
                }
                s.last_error = Some(e.to_string());
            }
        }
        state.lock().unwrap().connected = false;
        tokio::time::sleep(Duration::from_millis(700)).await;
    }
}

async fn session(addr: &str, state: &Arc<Mutex<LiviState>>) -> std::io::Result<()> {
    let s = TcpStream::connect(addr).await?;
    s.set_nodelay(true)?;
    let (mut r, mut w) = s.into_split();
    send(
        &mut w,
        &Request::Hello {
            version: carpi_proto::PROTOCOL_VERSION,
            client: "rnse-sim (LIVI stand-in)".into(),
        },
    )
    .await?;
    match read(&mut r).await? {
        Response::Hello { .. } => {}
        Response::Error { message } => return Err(std::io::Error::other(message)),
        other => return Err(std::io::Error::other(format!("unexpected {other:?}"))),
    }
    send(&mut w, &Request::Commands).await?;
    let mut prefixes: Vec<String> = LIVI_PREFIXES.iter().map(|s| s.to_string()).collect();
    prefixes.extend(EXTRA_PREFIXES.iter().map(|s| s.to_string()));
    send(
        &mut w,
        &Request::Subscribe {
            sub: 1,
            prefixes,
            period_ms: 200,
        },
    )
    .await?;
    {
        let mut s = state.lock().unwrap();
        s.connected = true;
        s.last_error = None;
        s.values.clear();
        s.event("bridge: connected to carpid".into());
    }

    let (tx, mut rx) = tokio::sync::mpsc::channel::<Response>(256);
    let reader = tokio::spawn(async move {
        while let Ok(resp) = read(&mut r).await {
            if tx.send(resp).await.is_err() {
                break;
            }
        }
    });
    let mut sent: Option<Push> = None;
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    let result = loop {
        tokio::select! {
            resp = rx.recv() => {
                let Some(resp) = resp else {
                    break Err(std::io::Error::other("carpid closed the connection"));
                };
                if let Some(done) = handle(resp, state) {
                    send(&mut w, &done).await?;
                }
            }
            _ = tick.tick() => {
                let want = state.lock().unwrap().want.clone();
                for u in diff(sent.as_ref(), &want) {
                    send(&mut w, &Request::Bridge { update: Box::new(u) }).await?;
                }
                sent = Some(want);
            }
        }
    };
    reader.abort();
    result
}

/// Updates for what changed since `old` (everything when `None`).
fn diff(old: Option<&Push>, new: &Push) -> Vec<BridgeUpdate> {
    let mut out = Vec::new();
    if old.is_none_or(|o| o.projection != new.projection) {
        out.push(BridgeUpdate::Projection {
            projection: new.projection.clone(),
        });
    }
    if old.is_none_or(|o| o.nav != new.nav) {
        out.push(BridgeUpdate::Nav {
            nav: new.nav.clone(),
        });
    }
    if old.is_none_or(|o| o.media != new.media) {
        out.push(BridgeUpdate::Media {
            media: new.media.clone(),
        });
    }
    if old.is_none_or(|o| o.call != new.call) {
        out.push(BridgeUpdate::Call {
            call: new.call.clone(),
        });
    }
    if old.is_none_or(|o| o.focus != new.focus) {
        out.push(BridgeUpdate::Focus {
            focus: new.focus.clone(),
        });
    }
    out
}

/// Records a response; returns the `CommandDone` to send for a command.
fn handle(resp: Response, state: &Arc<Mutex<LiviState>>) -> Option<Request> {
    let mut s = state.lock().unwrap();
    match resp {
        Response::Command { id, command } => {
            match &command {
                Command::Input { action } => {
                    let keys = aa_keys(action);
                    if let InputAction::Rotate { steps } = action {
                        let now = Instant::now();
                        for _ in 0..steps.unsigned_abs().min(16) {
                            s.rotary.push_back((now, i64::from(steps.signum())));
                        }
                        while s.rotary.len() > 400 {
                            s.rotary.pop_front();
                        }
                    }
                    s.event(format!("AA ← {}  ⇒  {}", describe(action), keys.join(", ")));
                }
                Command::Toast { text, .. } => s.event(format!("toast: {text}")),
                Command::Ap { ap } => s.event(format!("hotspot {ap:?} (ignored)")),
            }
            Some(Request::CommandDone { id, error: None })
        }
        Response::Update { items, .. } => {
            for r in items {
                s.values.insert(r.name, value_text(&r.value));
            }
            None
        }
        Response::Error { message } => {
            s.event(format!("carpid error: {message}"));
            None
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotary_is_one_event_per_step() {
        assert_eq!(
            aa_keys(&InputAction::Rotate { steps: 3 }),
            vec!["ROTARY +1", "ROTARY +1", "ROTARY +1"]
        );
        assert_eq!(aa_keys(&InputAction::Rotate { steps: -40 }).len(), 16);
        assert_eq!(
            aa_keys(&InputAction::Select { down: true }),
            vec!["DPAD_CENTER down"]
        );
    }

    #[test]
    fn diff_sends_all_first_then_changes() {
        let p = Push {
            focus: "livi".into(),
            ..Default::default()
        };
        assert_eq!(diff(None, &p).len(), 5);
        let mut q = p.clone();
        q.focus = "projection".into();
        assert_eq!(
            diff(Some(&p), &q),
            vec![BridgeUpdate::Focus {
                focus: "projection".into()
            }]
        );
    }
}
