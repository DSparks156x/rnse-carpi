//! `--selftest`: the whole loop without a window. Real carpid, real frames;
//! checks that each key reaches what LIVI or Android Auto would get, that the
//! cluster gets drawn, and that car values arrive where LIVI reads them.

use crate::keys::{AnyKey, Panel, Wheel};
use crate::{Sim, cluster, hub, livi, png, process};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

struct T<'a> {
    sim: &'a Arc<Mutex<Sim>>,
    livi: &'a Arc<Mutex<livi::LiviState>>,
    carpid: &'a Arc<Mutex<process::CarpidView>>,
    pass: u32,
    fail: u32,
}

impl T<'_> {
    fn check(&mut self, name: &str, ok: bool, detail: String) {
        if ok {
            self.pass += 1;
            println!("PASS  {name}  {detail}");
        } else {
            self.fail += 1;
            println!("FAIL  {name}  {detail}");
        }
    }

    fn wait(&self, timeout: Duration, mut f: impl FnMut() -> bool) -> bool {
        let end = Instant::now() + timeout;
        while Instant::now() < end {
            if f() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        f()
    }

    fn value(&self, name: &str) -> Option<String> {
        self.livi.lock().unwrap().values.get(name).cloned()
    }

    /// Events (AA commands and carpid routes) since `t`.
    fn events_since(&self, t: Instant) -> Vec<String> {
        let mut v: Vec<(Instant, String)> = self
            .livi
            .lock()
            .unwrap()
            .events
            .iter()
            .filter(|(at, _)| *at >= t)
            .cloned()
            .collect();
        v.extend(
            self.carpid
                .lock()
                .unwrap()
                .routes
                .iter()
                .filter(|(at, _)| *at >= t)
                .cloned(),
        );
        v.sort_by_key(|(at, _)| *at);
        v.into_iter().map(|(_, e)| e).collect()
    }

    fn tap(&self, key: AnyKey, ms: u64) {
        let now = Instant::now();
        let mut s = self.sim.lock().unwrap();
        let Sim { keys, bus, car, .. } = &mut *s;
        keys.tap(key, Duration::from_millis(ms), now, bus, car);
    }

    /// Tap `key` for `ms`, then wait for an event containing `want`.
    fn key_case(&mut self, name: &str, key: AnyKey, ms: u64, want: &[&str]) {
        let t = Instant::now();
        self.tap(key, ms);
        let ok = self.wait(Duration::from_millis(ms + 1500), || {
            let ev = self.events_since(t);
            want.iter().all(|w| ev.iter().any(|e| e.contains(w)))
        });
        let ev = self.events_since(t);
        self.check(name, ok, format!("→ {}", ev.join(" | ")));
    }

    fn snapshot(&self, work: &Path, name: &str) {
        let s = self.sim.lock().unwrap();
        let c = &s.cluster;
        let centre = png::encode(c.centre(), cluster::CENTRE_W, cluster::CENTRE_H, 4);
        let top = png::encode(
            &c.top_pixels(),
            cluster::TOP_CELLS * cluster::CELL_W,
            cluster::CELL_H * 2,
            4,
        );
        let _ = std::fs::write(work.join(format!("dis-{name}.png")), centre);
        let _ = std::fs::write(work.join(format!("dis-{name}-top.png")), top);
    }
}

pub fn run(
    _rt: &tokio::runtime::Runtime,
    sim: &Arc<Mutex<Sim>>,
    livi_state: &Arc<Mutex<livi::LiviState>>,
    carpid: &Arc<Mutex<process::CarpidView>>,
    hub_status: &Arc<Mutex<hub::HubStatus>>,
    work: &Path,
) -> bool {
    let mut t = T {
        sim,
        livi: livi_state,
        carpid,
        pass: 0,
        fail: 0,
    };

    let up = t.wait(Duration::from_secs(20), || {
        let h = hub_status.lock().unwrap();
        h.rx >= 1 && h.tx >= 1 && livi_state.lock().unwrap().connected
    });
    {
        let h = hub_status.lock().unwrap();
        let status = carpid.lock().unwrap().status.clone();
        t.check(
            "carpid on the virtual bus, LIVI stand-in on its API",
            up,
            format!("rx {} tx {} · {status}", h.rx, h.tx),
        );
    }
    if !up {
        for l in carpid.lock().unwrap().lines.iter().rev().take(15) {
            println!("      carpid: {l}");
        }
        return false;
    }

    let drawn = t.wait(Duration::from_secs(15), || {
        let s = sim.lock().unwrap();
        s.cluster.owned() && s.cluster.messages > 5
    });
    let shown = t.value("dis.shown");
    t.check(
        "DIS: carpid claims the centre and draws",
        drawn,
        format!(
            "{} messages, dis.shown = {shown:?}",
            sim.lock().unwrap().cluster.messages
        ),
    );
    std::thread::sleep(Duration::from_millis(800));
    t.snapshot(work, "readings");

    // Projection focus: carpid sends the keys as AA actions over the bridge.
    let focused = t.wait(Duration::from_secs(5), || {
        t.value("ui.focus").as_deref() == Some("projection")
    });
    t.check("focus = projection reaches carpid", focused, String::new());
    use Panel::*;
    t.key_case(
        "NEXT short → AA next track",
        AnyKey::Panel(Next),
        150,
        &["MEDIA_NEXT"],
    );
    t.key_case(
        "NEXT long (800 ms) → AA D-pad right",
        AnyKey::Panel(Next),
        800,
        &["DPAD_RIGHT down", "DPAD_RIGHT up"],
    );
    t.key_case(
        "PREV long → AA D-pad left",
        AnyKey::Panel(Prev),
        800,
        &["DPAD_LEFT down"],
    );
    t.key_case("UP → AA D-pad up", AnyKey::Panel(Up), 150, &["DPAD_UP"]);
    t.key_case(
        "DOWN → AA D-pad down",
        AnyKey::Panel(Down),
        150,
        &["DPAD_DOWN"],
    );
    t.key_case(
        "knob push → AA DPAD_CENTER down/up",
        AnyKey::Panel(Enter),
        150,
        &["DPAD_CENTER down", "DPAD_CENTER up"],
    );
    t.key_case(
        "RETURN → AA BACK",
        AnyKey::Panel(Return),
        150,
        &["BACK down"],
    );
    t.key_case(
        "RETURN long → AA HOME",
        AnyKey::Panel(Return),
        800,
        &["HOME"],
    );
    t.key_case(
        "NAV → AA navigation",
        AnyKey::Panel(Nav),
        150,
        &["NAVIGATION"],
    );
    t.key_case(
        "wheel click → AA play/pause",
        AnyKey::Wheel(Wheel::Click),
        300,
        &["MEDIA_PLAY_PAUSE"],
    );

    for (steps, rate) in [(3, 20.0), (-2, 20.0), (10, 40.0)] {
        let start = Instant::now();
        sim.lock().unwrap().keys.knob(steps, rate, start);
        let want = steps.unsigned_abs() as usize;
        let tag = if steps > 0 { "rotate +1" } else { "rotate -1" };
        let ok = t.wait(Duration::from_secs(3), || {
            t.events_since(start)
                .iter()
                .filter(|e| e.contains(tag))
                .count()
                >= want
        });
        std::thread::sleep(Duration::from_millis(300));
        let got = t
            .events_since(start)
            .iter()
            .filter(|e| e.contains(tag))
            .count();
        t.check(
            &format!("knob {steps:+} detents at {rate}/s → AA rotary"),
            ok && got == want,
            format!("{got} × {tag} (one per detent, never a bigger delta)"),
        );
    }

    // LIVI's own menus: keys go to uinput (logged by carpid off Linux).
    livi_state.lock().unwrap().want.focus = "livi".into();
    let ok = t.wait(Duration::from_secs(3), || {
        t.value("ui.focus").as_deref() == Some("livi")
    });
    t.check("focus = livi reaches carpid", ok, String::new());
    let start = Instant::now();
    sim.lock().unwrap().keys.knob(1, 50.0, start);
    let ok = t.wait(Duration::from_secs(3), || {
        t.events_since(start)
            .iter()
            .any(|e| e.contains("LIVI ← KnobRight"))
    });
    t.check(
        "knob in LIVI's menus → uinput PageDown",
        ok,
        t.events_since(start).join(" | "),
    );
    t.key_case(
        "NEXT long in LIVI → uinput Right",
        AnyKey::Panel(Next),
        800,
        &["LIVI ← Right"],
    );
    livi_state.lock().unwrap().want.focus = "projection".into();

    // Off TV, carpid drops the faceplate.
    sim.lock().unwrap().car.on_tv = false;
    t.wait(Duration::from_secs(2), || {
        t.value("rnse.on_tv").as_deref() == Some("false")
    });
    let start = Instant::now();
    t.tap(AnyKey::Panel(Next), 150);
    std::thread::sleep(Duration::from_millis(900));
    let ev = t.events_since(start);
    t.check(
        "RNS-E not on TV → keys dropped",
        !ev.iter().any(|e| e.contains("MEDIA_NEXT")),
        ev.join(" | "),
    );
    sim.lock().unwrap().car.on_tv = true;

    // Car values where LIVI reads them.
    {
        let mut s = sim.lock().unwrap();
        s.car.speed_kmh = 87.0;
        s.car.steer_deg = -45.0;
        s.car.gear = crate::car::Gear::R;
        s.car.door_fl = true;
        s.car.set_night(true);
    }
    let ok = t.wait(Duration::from_secs(4), || {
        t.value("vehicle.speed_kmh").as_deref() == Some("87")
            && t.value("trans.selector").as_deref() == Some("7")
            && t.value("body.door_fl_open").as_deref() == Some("true")
            && t.value("lights.low_beam").as_deref() == Some("true")
            && t.value("steering.angle_deg")
                .is_some_and(|v| v.starts_with("-45"))
    });
    t.check(
        "speed, gear, steering, door, lights → LIVI's values",
        ok,
        format!(
            "speed {:?} selector {:?} steer {:?} door {:?} low {:?} night {:?}",
            t.value("vehicle.speed_kmh"),
            t.value("trans.selector"),
            t.value("steering.angle_deg"),
            t.value("body.door_fl_open"),
            t.value("lights.low_beam"),
            t.value("light.night"),
        ),
    );
    {
        let mut s = sim.lock().unwrap();
        s.car.speed_kmh = 0.0;
        s.car.steer_deg = 0.0;
        s.car.gear = crate::car::Gear::P;
        s.car.door_fl = false;
    }

    // Nav and media take the centre.
    livi_state.lock().unwrap().want.nav = Some(carpi_proto::NavState {
        maneuver: Some("turn".into()),
        side: Some("right".into()),
        distance_m: Some(350.0),
        distance_unit: Some("m".into()),
        road: Some("Hauptstrasse".into()),
        destination_distance_m: Some(12_400.0),
        time_to_destination_s: Some(840.0),
        eta: Some("17:42".into()),
        ..Default::default()
    });
    let ok = t.wait(Duration::from_secs(8), || {
        t.value("dis.shown").as_deref() == Some("nav")
    });
    t.check(
        "nav guidance → DIS nav screen",
        ok,
        format!("dis.shown = {:?}", t.value("dis.shown")),
    );
    std::thread::sleep(Duration::from_millis(1500));
    t.snapshot(work, "nav");
    livi_state.lock().unwrap().want.nav = None;
    livi_state.lock().unwrap().want.media = Some(carpi_proto::MediaState {
        source: Some("Android Auto".into()),
        title: Some("Test Track".into()),
        artist: Some("rnse-sim".into()),
        album: None,
        playing: true,
        position_ms: Some(30_000),
        duration_ms: Some(215_000),
    });
    let ok = t.wait(Duration::from_secs(8), || {
        t.value("dis.shown").as_deref() == Some("media")
    });
    t.check(
        "media → DIS media screen",
        ok,
        format!("dis.shown = {:?}", t.value("dis.shown")),
    );
    std::thread::sleep(Duration::from_millis(1500));
    t.snapshot(work, "media");

    // Nav over media: the centre shows nav, the top lines carry the track.
    let nav = carpi_proto::NavState {
        maneuver: Some("roundabout_enter_and_exit".into()),
        side: Some("right".into()),
        roundabout_exit: Some(2),
        roundabout_angle: Some(90),
        distance_m: Some(120.0),
        distance_unit: Some("m".into()),
        road: Some("Ring".into()),
        ..Default::default()
    };
    livi_state.lock().unwrap().want.nav = Some(nav);
    let blank = "20 20 20 20 20 20 20 20";
    let ok = t.wait(Duration::from_secs(8), || {
        let hex = sim.lock().unwrap().cluster.top_hex();
        t.value("dis.shown").as_deref() == Some("nav")
            && !hex.starts_with(blank)
            && !hex.starts_with('-')
    });
    let hex = sim.lock().unwrap().cluster.top_hex();
    t.check(
        "top lines (0x363) carry media while nav has the centre",
        ok,
        hex,
    );
    std::thread::sleep(Duration::from_millis(1500));
    t.snapshot(work, "nav-roundabout");
    livi_state.lock().unwrap().want.nav = None;

    let (label, brightness, violations) = {
        let s = sim.lock().unwrap();
        (
            s.rnse_label,
            s.rnse_brightness,
            s.cluster.violations.clone(),
        )
    };
    t.check(
        "RNS-E label/brightness (0x7B0 BA/BB) sent",
        label.is_some(),
        format!("BA {label:?} BB {brightness:?}"),
    );
    t.check(
        "cluster saw no protocol violations",
        violations.is_empty() && !sim.lock().unwrap().cluster.frozen(),
        violations.join("; "),
    );

    println!(
        "\n{} passed, {} failed. DIS snapshots: {}",
        t.pass,
        t.fail,
        work.join("dis-*.png").display()
    );
    t.fail == 0
}

/// With the real LIVI: carpid and LIVI find each other, and the keys carpid
/// routes to LIVI's menus and pages arrive (and are typed into its window).
pub fn run_real(
    sim: &Arc<Mutex<Sim>>,
    carpid: &Arc<Mutex<process::CarpidView>>,
    livi_view: &Arc<Mutex<process::CarpidView>>,
    hub_status: &Arc<Mutex<hub::HubStatus>>,
) -> bool {
    let quiet = Arc::new(Mutex::new(livi::LiviState::default()));
    let mut t = T {
        sim,
        livi: &quiet,
        carpid,
        pass: 0,
        fail: 0,
    };
    let up = t.wait(Duration::from_secs(20), || {
        let h = hub_status.lock().unwrap();
        h.rx >= 1 && h.tx >= 1
    });
    t.check(
        "carpid on the virtual bus",
        up,
        carpid.lock().unwrap().status.clone(),
    );
    let drawn = t.wait(Duration::from_secs(15), || {
        let s = sim.lock().unwrap();
        s.cluster.owned() && s.cluster.messages > 5
    });
    t.check("DIS drawn", drawn, String::new());

    // LIVI takes a few seconds; carpid routes keys once LIVI has pushed state.
    let start = Instant::now();
    let connected = t.wait(Duration::from_secs(40), || {
        if t.events_since(start)
            .iter()
            .any(|e| e.starts_with("LIVI ←"))
        {
            return true;
        }
        t.tap(AnyKey::Panel(Panel::Down), 120);
        std::thread::sleep(Duration::from_millis(700));
        false
    });
    let livi_log: Vec<String> = livi_view
        .lock()
        .unwrap()
        .lines
        .iter()
        .rev()
        .take(6)
        .cloned()
        .collect();
    t.check(
        "LIVI connected to carpid; keys route to LIVI",
        connected,
        format!(
            "{} · last LIVI lines: {}",
            livi_view.lock().unwrap().status,
            livi_log.join(" | ")
        ),
    );
    if !connected {
        return false;
    }
    let start = Instant::now();
    sim.lock().unwrap().keys.knob(3, 10.0, start);
    let ok = t.wait(Duration::from_secs(4), || {
        t.events_since(start)
            .iter()
            .filter(|e| e.contains("LIVI ← KnobRight"))
            .count()
            >= 3
    });
    t.check(
        "knob +3 → LIVI KnobRight ×3 (typed as PageDown)",
        ok,
        t.events_since(start).join(" | "),
    );
    t.key_case(
        "knob push → LIVI Select (Enter)",
        AnyKey::Panel(Panel::Enter),
        150,
        &["LIVI ← Select"],
    );
    t.key_case(
        "RETURN → LIVI Back (Backspace)",
        AnyKey::Panel(Panel::Return),
        150,
        &["LIVI ← Back"],
    );
    let start = Instant::now();
    t.tap(AnyKey::Panel(Panel::Car), 150);
    std::thread::sleep(Duration::from_millis(1500));
    t.tap(AnyKey::Panel(Panel::Down), 150);
    let ok = t.wait(Duration::from_secs(4), || {
        t.events_since(start).iter().any(|e| e.contains("page:car"))
    });
    t.check(
        "CAR → LIVI opens its Car page; focus page:car comes back",
        ok,
        t.events_since(start).join(" | "),
    );
    // Room for LIVI_UI_SNAPSHOT.
    std::thread::sleep(Duration::from_secs(4));
    for l in livi_view
        .lock()
        .unwrap()
        .lines
        .iter()
        .filter(|l| l.contains("[ui]"))
    {
        println!("      {l}");
    }
    println!(
        "
{} passed, {} failed",
        t.pass, t.fail
    );
    t.fail == 0
}
