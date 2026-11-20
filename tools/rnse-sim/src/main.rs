//! rnse-sim: the car around a real carpid, on the desktop.
//!
//! ```text
//!  UI ─▶ keys/car ─▶ frames ─▶ hub (TCP virtual CAN) ─▶ carpid (CARPI_SIM_CAN)
//!                                ▲                        │ DIS, top lines, 0x7B0
//!  cluster (SimCluster + Fb) ◀───┴────────────────────────┘
//!  LIVI stand-in ◀──── carpid API (tcp) ────▶ focus, nav, media; AA commands
//! ```
//!
//! carpid runs unmodified apart from where its frames go: its decoder,
//! gesture timing, routing, DIS driver and TX gate all run as in the car.
//! See `docs/SIM.md`.

mod car;
mod cluster;
mod hub;
mod keys;
mod livi;
mod png;
mod process;
mod selftest;

use anyhow::Context;
use carpi_can::Frame;
use clap::Parser;
use slint::{ComponentHandle, Model};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

slint::include_modules!();

#[derive(Parser)]
#[command(about = "Virtual RNS-E, cluster and car around a real carpid")]
struct Args {
    /// Virtual CAN hub address (carpid gets it as CARPI_SIM_CAN).
    #[arg(long, default_value = "127.0.0.1:29536")]
    hub: String,
    /// carpid's API, as in the sim config's `[api] socket = "tcp:..."`.
    #[arg(long, default_value = "127.0.0.1:29537")]
    api: String,
    /// The carpid binary. Default: the workspace's debug build.
    #[arg(long)]
    carpid: Option<PathBuf>,
    /// Don't start carpid; one is already running against this hub (e.g.
    /// in WSL with CARPI_SIM_CAN pointing here).
    #[arg(long)]
    external_carpid: bool,
    /// Don't act as LIVI on carpid's API: the real LIVI connects instead.
    #[arg(long)]
    no_livi: bool,
    /// Run the real LIVI (Linux): this `livi-core` binary, connected to
    /// carpid's unix socket. Implies --no-livi. Under WSL the keys carpid
    /// routes to LIVI's menus are typed into its window from Windows.
    #[arg(long)]
    livi: Option<PathBuf>,
    /// Working folder: carpid.toml, its home, state and logs.
    #[arg(long)]
    work: Option<PathBuf>,
    /// No window: drive carpid through every path, print PASS/FAIL, write
    /// the cluster's screens as PNGs to the work folder, and exit.
    #[arg(long)]
    selftest: bool,
    /// With the window: save it to this PNG after a few seconds, then quit.
    #[arg(long, hide = true)]
    snapshot: Option<PathBuf>,
}

/// The car side: everything on the bus that isn't carpid.
pub struct Sim {
    pub car: car::CarState,
    pub bus: car::CarBus,
    pub keys: keys::Keys,
    pub cluster: cluster::Cluster,
    /// 0x7B0 commands carpid sent the RNS-E: label (BA) and brightness (BB).
    pub rnse_label: Option<u8>,
    pub rnse_brightness: Option<u8>,
    /// Recent carpid frames other than DDP, newest last.
    pub can_log: VecDeque<String>,
    pub frames_in: u64,
    pub frames_out: u64,
}

impl Sim {
    fn new(now: Instant) -> Self {
        Sim {
            car: car::CarState::default(),
            bus: car::CarBus::new(now),
            keys: keys::Keys::new(now),
            cluster: cluster::Cluster::new(),
            rnse_label: None,
            rnse_brightness: None,
            can_log: VecDeque::new(),
            frames_in: 0,
            frames_out: 0,
        }
    }

    /// A frame carpid sent.
    pub fn from_carpid(&mut self, now: Instant, f: &Frame) {
        self.frames_in += 1;
        self.cluster.on_frame(now, f);
        if f.id == carpi_signals::ids::RNSE_CMD && f.len() >= 2 {
            match f.data()[0] {
                0xBA => self.rnse_label = Some(f.data()[1]),
                0xBB => self.rnse_brightness = Some(f.data()[1]),
                _ => {}
            }
        }
        if f.id != carpi_signals::ids::DDP_TX {
            let data: Vec<String> = f.data().iter().map(|b| format!("{b:02X}")).collect();
            self.can_log
                .push_back(format!("{:03X} {}", f.id, data.join(" ")));
            while self.can_log.len() > 6 {
                self.can_log.pop_front();
            }
        }
    }

    /// Everything due on the bus from our side.
    fn poll(&mut self, now: Instant, out: &mut Vec<Frame>) {
        let Sim {
            car,
            bus,
            keys,
            cluster,
            ..
        } = self;
        bus.poll(car, now, out);
        keys.poll(now, bus, car, out);
        if car.awake {
            cluster.tick(now, out);
        }
        self.frames_out += out.len() as u64;
    }
}

pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    #[cfg(windows)]
    // 1 ms timers, so the 110 ms key repeats and DDP ACKs keep their timing.
    unsafe {
        windows_sys::Win32::Media::timeBeginPeriod(1);
    }

    let work = args
        .work
        .clone()
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("work"));
    std::fs::create_dir_all(&work).with_context(|| format!("creating {}", work.display()))?;
    let real_livi = args.livi.is_some();
    let unix_socket = work.join("run").join("carpid.sock");
    let socket = if real_livi {
        unix_socket.display().to_string()
    } else {
        format!("tcp:{}", args.api)
    };
    let config = process::ensure_config(&work, &socket)?;
    for d in ["home", "state", "run"] {
        std::fs::create_dir_all(work.join(d))?;
    }

    let rt = tokio::runtime::Runtime::new()?;
    let now = Instant::now();
    let sim = Arc::new(Mutex::new(Sim::new(now)));
    let livi_state = Arc::new(Mutex::new(livi::LiviState {
        want: livi::Push {
            focus: "projection".into(),
            projection: Some(carpi_proto::Projection {
                kind: "android_auto".into(),
                wireless: false,
            }),
            ..Default::default()
        },
        ..Default::default()
    }));
    let carpid_view = Arc::new(Mutex::new(process::CarpidView::default()));

    let hub = rt.block_on(hub::Hub::bind(&args.hub))?;
    let hub_status = hub.status();
    rt.spawn(hub.clone().run(sim.clone()));
    rt.spawn(sim_loop(sim.clone(), hub.clone()));
    if !args.no_livi && !real_livi {
        rt.spawn(livi::run(args.api.clone(), livi_state.clone()));
    }
    let carpid_bin = args.carpid.clone().unwrap_or_else(|| {
        repo_root()
            .join("target/debug")
            .join(format!("carpid{}", std::env::consts::EXE_SUFFIX))
    });
    let os = |p: &std::path::Path| p.as_os_str().to_owned();
    let runner = process::Runner {
        label: "carpid",
        bin: carpid_bin,
        args: vec!["--config".into(), os(&config)],
        env: vec![
            ("CARPI_SIM_CAN".into(), args.hub.clone().into()),
            ("HOME".into(), os(&work.join("home"))),
            ("USERPROFILE".into(), os(&work.join("home"))),
            ("XDG_STATE_HOME".into(), os(&work.join("state"))),
            ("XDG_RUNTIME_DIR".into(), os(&work.join("run"))),
            ("NO_COLOR".into(), "1".into()),
            (
                "RUST_LOG".into(),
                std::env::var("RNSE_SIM_LOG")
                    .unwrap_or_else(|_| "info,carpid::act=debug".into())
                    .into(),
            ),
        ],
        env_remove: Vec::new(),
        work: work.clone(),
        xkeys: real_livi && std::env::var_os("WSL_DISTRO_NAME").is_some(),
        view: carpid_view.clone(),
    };
    let livi_view = Arc::new(Mutex::new(process::CarpidView::default()));
    let livi_runner = args.livi.clone().map(|bin| process::Runner {
        label: "livi",
        args: Vec::new(),
        env: {
            // A cargo build keeps LIVI's binaries side by side; tell core where
            // (its own lookup wants `out/bin` under the checkout).
            let dir = bin
                .parent()
                .map(std::path::Path::to_path_buf)
                .unwrap_or_default();
            vec![
                ("LIVI_ROOT".into(), os(&repo_root().join("livi"))),
                ("LIVI_UI_CMD".into(), os(&dir.join("LIVI"))),
                ("LIVI_GST_HOST_BIN".into(), os(&dir.join("livi-gst-host"))),
                (
                    "LIVI_COMPOSITOR_BIN".into(),
                    os(&dir.join("livi-compositor")),
                ),
                ("LIVI_HELPER_BIN".into(), os(&dir.join("livi-helperd"))),
                ("CARPID_SOCKET".into(), os(&unix_socket)),
                // Its own config, seeded as the car's first boot does.
                ("XDG_CONFIG_HOME".into(), os(&work.join("livi-config"))),
                // The RNS-E's TV input as LIVI is set up for it (SDLink, 480 EDIDs).
                ("LIVI_UI_SIZE".into(), "800x480".into()),
                // Straight onto the desktop's Wayland (WSLg): LIVI's nested
                // compositor has no display to sit on there.
                ("LIVI_NO_COMPOSITOR".into(), "1".into()),
                ("NO_COLOR".into(), "1".into()),
            ]
        },
        bin,
        env_remove: Vec::new(),
        work: work.clone(),
        xkeys: false,
        view: livi_view.clone(),
    });
    let restart = rt.handle().clone();
    if args.selftest {
        if !args.external_carpid {
            runner.start(&restart);
        }
        let ok = match &livi_runner {
            Some(l) => {
                seed_livi_config(&work.join("livi-config"))?;
                l.start(&restart);
                selftest::run_real(&sim, &carpid_view, &livi_view, &hub_status)
            }
            None => selftest::run(&rt, &sim, &livi_state, &carpid_view, &hub_status, &work),
        };
        process::stop_all(&carpid_view);
        process::stop_all(&livi_view);
        drop(rt);
        std::process::exit(if ok { 0 } else { 1 });
    }
    if livi_runner.is_some() {
        seed_livi_config(&work.join("livi-config"))?;
    }
    if let Some(l) = &livi_runner {
        l.start(&restart);
    }
    if !args.external_carpid {
        runner.start(&restart);
    } else {
        carpid_view.lock().unwrap().status = format!(
            "external carpid: start it with CARPI_SIM_CAN={} and [api] socket = \"tcp:{}\"",
            args.hub, args.api
        );
    }

    let ui = SimWindow::new()?;
    ui.set_real_livi(real_livi);
    wire_ui(
        &ui,
        &sim,
        &livi_state,
        &carpid_view,
        runner,
        restart,
        args.external_carpid,
    );

    let timer = slint::Timer::default();
    {
        let ui = ui.as_weak();
        let sim = sim.clone();
        let livi_state = livi_state.clone();
        let carpid_view = carpid_view.clone();
        let livi_view = livi_view.clone();
        timer.start(
            slint::TimerMode::Repeated,
            Duration::from_millis(50),
            move || {
                if let Some(ui) = ui.upgrade() {
                    refresh(&ui, &sim, &livi_state, &carpid_view, &hub_status);
                    if real_livi {
                        show_livi(&ui, &livi_view);
                    }
                }
            },
        );
    }
    let snap_timer = slint::Timer::default();
    if let Some(path) = args.snapshot.clone() {
        let ui = ui.as_weak();
        let sim = sim.clone();
        snap_timer.start(
            slint::TimerMode::Repeated,
            Duration::from_secs(1),
            move || {
                let Some(ui) = ui.upgrade() else { return };
                // Wait for the cluster to be drawn, at most ~15 s.
                let n = SNAP_TICKS.with(|c| {
                    c.set(c.get() + 1);
                    c.get()
                });
                if n < 8 || (!sim.lock().unwrap().cluster.owned() && n < 20) {
                    return;
                }
                if let Ok(buf) = ui.window().take_snapshot() {
                    let png = png::encode_rgba(
                        buf.as_bytes(),
                        buf.width() as usize,
                        buf.height() as usize,
                    );
                    let _ = std::fs::write(&path, png);
                }
                let _ = slint::quit_event_loop();
            },
        );
    }
    ui.run()?;
    carpid_view.lock().unwrap().quit = true;
    livi_view.lock().unwrap().quit = true;
    process::stop_all(&carpid_view);
    process::stop_all(&livi_view);
    Ok(())
}

/// LIVI's config.json for the sim, as the car's first boot sets it up
/// (`os/options.py` `apply_user_config`): the knob on PageUp/PageDown (LIVI's
/// own default binds nothing, so the knob does nothing in its menus) and the
/// RNS-E's 800x480, and knob keys declared to Android Auto. The first time it starts from `~/.config/LIVI/config.json`
/// if there is one, so settings made in earlier runs carry over. Each start
/// fills empty knob bindings and leaves set ones alone.
fn seed_livi_config(dir: &std::path::Path) -> anyhow::Result<()> {
    let path = dir.join("LIVI").join("config.json");
    std::fs::create_dir_all(path.parent().expect("has a parent"))?;
    let mut cfg: serde_json::Value = std::fs::read_to_string(&path)
        .ok()
        .or_else(|| {
            let home = std::env::var_os("HOME")?;
            std::fs::read_to_string(PathBuf::from(home).join(".config/LIVI/config.json")).ok()
        })
        .and_then(|t| serde_json::from_str(&t).ok())
        .filter(serde_json::Value::is_object)
        .unwrap_or_else(|| {
            serde_json::json!({
                "mainScreenWidth": 800,
                "mainScreenHeight": 480,
                "projectionWidth": 800,
                "projectionHeight": 480,
            })
        });
    let before = cfg.clone();
    let bindings = cfg
        .as_object_mut()
        .expect("checked")
        .entry("bindings")
        .or_insert_with(|| serde_json::json!({}));
    if let Some(b) = bindings.as_object_mut() {
        for (name, key) in [("knobLeft", "PageUp"), ("knobRight", "PageDown")] {
            let empty = b
                .get(name)
                .and_then(|v| v.as_str())
                .is_none_or(str::is_empty);
            if empty {
                b.insert(name.into(), key.into());
            }
        }
    }
    // The RNS-E declares knob keys to Android Auto, as the car's seed does.
    let aa = cfg
        .as_object_mut()
        .expect("checked")
        .entry("androidAuto")
        .or_insert_with(|| serde_json::json!({}));
    if let Some(aa) = aa.as_object_mut() {
        aa.entry("keys").or_insert_with(|| "rotary".into());
    }
    if cfg != before || !path.exists() {
        std::fs::write(
            &path,
            serde_json::to_string_pretty(&cfg)?
                + "
",
        )?;
    }
    Ok(())
}

/// The real LIVI's status and log in place of the stand-in's values.
fn show_livi(ui: &SimWindow, view: &Arc<Mutex<process::CarpidView>>) {
    let v = view.lock().unwrap();
    let start = v.lines.len().saturating_sub(200);
    let log: String = v
        .lines
        .iter()
        .skip(start)
        .rev()
        .map(|l| format!("{l}\n"))
        .collect();
    ui.set_values(log.into());
    ui.set_livi_status(v.status.clone().into());
}

/// Our side of the bus, every few ms.
async fn sim_loop(sim: Arc<Mutex<Sim>>, hub: Arc<hub::Hub>) {
    let mut tick = tokio::time::interval(Duration::from_millis(2));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut out = Vec::new();
    loop {
        tick.tick().await;
        let now = Instant::now();
        out.clear();
        sim.lock().unwrap().poll(now, &mut out);
        for f in &out {
            hub.broadcast("ican", f, None);
        }
    }
}

fn wire_ui(
    ui: &SimWindow,
    sim: &Arc<Mutex<Sim>>,
    livi_state: &Arc<Mutex<livi::LiviState>>,
    carpid_view: &Arc<Mutex<process::CarpidView>>,
    runner: process::Runner,
    rt: tokio::runtime::Handle,
    external: bool,
) {
    {
        let sim = sim.clone();
        ui.on_key_edge(move |name, down| {
            let Some(k) = keys::AnyKey::from_name(&name) else {
                return;
            };
            let now = Instant::now();
            let mut s = sim.lock().unwrap();
            let Sim { keys, bus, car, .. } = &mut *s;
            if down {
                keys.down(k, now, bus, car);
            } else {
                keys.up(k, now, bus, car);
            }
        });
    }
    {
        let sim = sim.clone();
        let wheel = std::cell::RefCell::new(keys::ScrollWheel::default());
        ui.on_knob_scroll(move |dy| {
            let now = Instant::now();
            let steps = wheel.borrow_mut().feed(dy, now);
            if steps != 0 {
                // Sent at once: the scroll speed is the knob speed.
                sim.lock().unwrap().keys.knob(steps, 200.0, now);
            }
        });
    }
    {
        let sim = sim.clone();
        let ui_weak = ui.as_weak();
        ui.on_knob_spin(move |steps| {
            let rate = ui_weak
                .upgrade()
                .map(|u| f64::from(u.get_spin_rate()))
                .unwrap_or(12.0);
            // Keyboard single steps go out at once (the OS key repeat is the rate).
            let rate = if steps.abs() == 1 { 200.0 } else { rate };
            sim.lock().unwrap().keys.knob(steps, rate, Instant::now());
        });
    }
    {
        let sim = sim.clone();
        ui.on_tap(move |name, ms| {
            let Some(k) = keys::AnyKey::from_name(&name) else {
                return;
            };
            let now = Instant::now();
            let mut s = sim.lock().unwrap();
            let Sim { keys, bus, car, .. } = &mut *s;
            keys.tap(k, Duration::from_millis(ms.max(10) as u64), now, bus, car);
        });
    }
    {
        let sim = sim.clone();
        ui.on_car_bool(move |name, v| {
            let mut s = sim.lock().unwrap();
            let c = &mut s.car;
            match name.as_str() {
                "awake" => c.awake = v,
                "kl15" => c.kl15 = v,
                "on_tv" => c.on_tv = v,
                "night" => c.set_night(v),
                "handbrake" => c.handbrake = v,
                "side" => c.side_lights = v,
                "low" => c.low_beam = v,
                "high" => c.high_beam = v,
                "fog" => c.fog_front = v,
                "turn_l" => c.turn_left = v,
                "turn_r" => c.turn_right = v,
                "hazard" => c.hazard = v,
                "door_fl" => c.door_fl = v,
                "door_fr" => c.door_fr = v,
                "door_rl" => c.door_rl = v,
                "door_rr" => c.door_rr = v,
                "hood" => c.hood = v,
                _ => {}
            }
        });
    }
    {
        let sim = sim.clone();
        ui.on_car_num(move |name, v| {
            let mut s = sim.lock().unwrap();
            match name.as_str() {
                "speed" => s.car.speed_kmh = f64::from(v),
                "steer" => s.car.steer_deg = f64::from(v),
                "knob-frames" => s.keys.knob_frames_per_detent = v.clamp(1.0, 4.0) as u8,
                _ => {}
            }
            // A moving car isn't parked, and runs its engine.
            if s.car.speed_kmh > 0.0 {
                s.car.rpm = 900.0 + s.car.speed_kmh * 22.0;
            } else {
                s.car.rpm = 800.0;
            }
        });
    }
    {
        let sim = sim.clone();
        ui.on_car_text(move |name, v| {
            let mut s = sim.lock().unwrap();
            match name.as_str() {
                "gear" => {
                    if let Some(g) = car::Gear::ALL.iter().find(|g| g.name() == v.as_str()) {
                        s.car.gear = *g;
                    }
                }
                "units" => s.car.us_units = v == "US",
                _ => {}
            }
        });
    }
    {
        let livi_state = livi_state.clone();
        let ui_weak = ui.as_weak();
        ui.on_livi(move |name, v| {
            let dist = ui_weak
                .upgrade()
                .map(|u| f64::from(u.get_nav_dist()))
                .unwrap_or(350.0);
            let mut s = livi_state.lock().unwrap();
            let w = &mut s.want;
            match name.as_str() {
                "focus" => w.focus = v.to_string(),
                "projection" => {
                    w.projection = (v != "none").then(|| carpi_proto::Projection {
                        kind: v.to_string(),
                        wireless: false,
                    })
                }
                "nav" => {
                    w.nav = (v == "on").then(|| carpi_proto::NavState {
                        maneuver: Some("turn".into()),
                        side: Some("right".into()),
                        distance_m: Some(dist),
                        distance_unit: Some("m".into()),
                        road: Some("Hauptstraße".into()),
                        destination_distance_m: Some(12_400.0),
                        time_to_destination_s: Some(840.0),
                        eta: Some("17:42".into()),
                        ..Default::default()
                    })
                }
                "maneuver" => {
                    if let Some(n) = &mut w.nav {
                        n.maneuver = Some(v.to_string());
                    }
                }
                "side" => {
                    if let Some(n) = &mut w.nav {
                        n.side = Some(v.to_string());
                    }
                }
                "distance" => {
                    if let Some(n) = &mut w.nav {
                        n.distance_m = v.parse().ok();
                    }
                }
                "media" => {
                    w.media = (v == "on").then(|| carpi_proto::MediaState {
                        source: Some("Android Auto".into()),
                        title: Some("Test Track".into()),
                        artist: Some("rnse-sim".into()),
                        album: Some("Bench".into()),
                        playing: true,
                        position_ms: Some(30_000),
                        duration_ms: Some(215_000),
                    })
                }
                "call" => {
                    w.call = (v != "none").then(|| carpi_proto::CallState {
                        state: v.to_string(),
                        number: Some("+49 30 1234567".into()),
                        name: Some("Bench".into()),
                    })
                }
                _ => {}
            }
        });
    }
    {
        let carpid_view = carpid_view.clone();
        let livi_state = livi_state.clone();
        ui.on_clear_log(move || {
            carpid_view.lock().unwrap().routes.clear();
            livi_state.lock().unwrap().events.clear();
        });
    }
    ui.on_restart_carpid(move || {
        if external {
            return;
        }
        runner.restart(&rt);
    });
}

fn image(px: &[bool], w: usize, h: usize, lit: [u8; 3]) -> slint::Image {
    let mut buf = slint::SharedPixelBuffer::<slint::Rgba8Pixel>::new(w as u32, h as u32);
    for (dst, on) in buf.make_mut_slice().iter_mut().zip(px) {
        *dst = if *on {
            slint::Rgba8Pixel::new(lit[0], lit[1], lit[2], 255)
        } else {
            slint::Rgba8Pixel::new(0, 0, 0, 255)
        };
    }
    slint::Image::from_rgba8(buf)
}

fn label_name(b: u8) -> &'static str {
    match b {
        0 => "LIVI",
        1 => "CarPlay",
        2 => "Android Auto",
        _ => "?",
    }
}

fn refresh(
    ui: &SimWindow,
    sim: &Arc<Mutex<Sim>>,
    livi_state: &Arc<Mutex<livi::LiviState>>,
    carpid_view: &Arc<Mutex<process::CarpidView>>,
    hub_status: &Arc<Mutex<hub::HubStatus>>,
) {
    let now = Instant::now();
    let lit = [0xF0, 0xF4, 0xFF];
    {
        let s = sim.lock().unwrap();
        let c = &s.cluster;
        ui.set_dis_centre(image(c.centre(), cluster::CENTRE_W, cluster::CENTRE_H, lit));
        ui.set_dis_top(image(
            &c.top_pixels(),
            cluster::TOP_CELLS * cluster::CELL_W,
            cluster::CELL_H * 2,
            lit,
        ));
        ui.set_dis_owned(c.owned());
        ui.set_dis_status(
            format!(
                "DDP {} · centre {} · {} drawing messages{}{}",
                if c.open() { "open" } else { "closed" },
                if c.owned() { "ours" } else { "cluster's own" },
                c.messages,
                if c.frozen() {
                    " · FROZEN (a message the real cluster freezes on)"
                } else {
                    ""
                },
                if c.has_top() {
                    ""
                } else {
                    " · no top lines yet"
                },
            )
            .into(),
        );
        ui.set_dis_notes(c.violations.join("\n").into());
        ui.set_rnse_label(
            format!(
                "label (BA): {} · brightness (BB): {}",
                s.rnse_label.map_or("not sent", label_name),
                s.rnse_brightness.map_or("not sent".into(), |b| if b == 0 {
                    "follow car".into()
                } else {
                    format!("{b}")
                }),
            )
            .into(),
        );
        let (sum, n) = s.keys.knob_rate(now);
        let aa: i64 = livi_state
            .lock()
            .unwrap()
            .rotary
            .iter()
            .filter(|(t, _)| now.saturating_duration_since(*t) <= Duration::from_secs(1))
            .map(|(_, d)| d)
            .sum();
        ui.set_knob_readout(
            format!(
                "last second: knob sent {n} detents (net {sum:+}); AA got {aa:+} rotary steps. carpid sends Rotate ±1 per detent, so AA never sees a delta above 1."
            )
            .into(),
        );
        ui.set_knob_angle(s.keys.position);
        let held_now = s.keys.held();
        let held: Vec<bool> = KEY_ORDER
            .iter()
            .map(|n| held_now.iter().any(|k| key_name(*k) == *n))
            .collect();
        let model = ui.get_held();
        if model.row_count() != held.len() || model.iter().zip(&held).any(|(a, b)| a != *b) {
            ui.set_held(slint::ModelRc::new(slint::VecModel::from(held)));
        }
        let mut cl = String::from("carpid → bus (non-DDP, newest last): ");
        cl.push_str(&s.can_log.iter().cloned().collect::<Vec<_>>().join(" | "));
        ui.set_can_log(cl.into());
        let hs = hub_status.lock().unwrap();
        ui.set_hub_status(
            format!(
                "virtual CAN {} · carpid connections: {} rx, {} tx · frames: {} to carpid, {} from carpid",
                hs.addr, hs.rx, hs.tx, s.frames_out, s.frames_in
            )
            .into(),
        );
    }
    let (events, last_aa, status) = {
        let l = livi_state.lock().unwrap();
        let ev: Vec<(Instant, String)> = l.events.iter().cloned().collect();
        let status = if l.connected {
            format!("LIVI stand-in connected · focus {}", l.want.focus)
        } else {
            format!(
                "LIVI stand-in not connected{}",
                l.last_error
                    .as_ref()
                    .map_or(String::new(), |e| format!(" ({e})"))
            )
        };
        let values: String = l
            .values
            .iter()
            .map(|(k, v)| format!("{k} = {v}\n"))
            .collect();
        ui.set_values(values.into());
        (ev, l.events.back().cloned(), status)
    };
    ui.set_livi_status(status.into());
    let cv = carpid_view.lock().unwrap();
    let mut all: Vec<(Instant, String)> = events;
    all.extend(cv.routes.iter().cloned());
    all.sort_by_key(|(t, _)| *t);
    let start = all.len().saturating_sub(40);
    let text: String = all[start..]
        .iter()
        .rev()
        .map(|(t, e)| {
            format!(
                "{:>6.1}s  {e}\n",
                now.saturating_duration_since(*t).as_secs_f32()
            )
        })
        .collect();
    ui.set_events(text.into());
    let last = all.last().cloned().or(last_aa);
    if let Some((t, e)) = last {
        ui.set_rnse_last(e.into());
        ui.set_rnse_sub(
            format!(
                "{:.1} s ago · LIVI's own UI isn't on Windows (docs/SIM.md); this shows where each key went",
                now.saturating_duration_since(t).as_secs_f32()
            )
            .into(),
        );
    } else {
        ui.set_rnse_last("press a key".into());
    }
    ui.set_carpid_status(cv.status.clone().into());
    let start = cv.lines.len().saturating_sub(200);
    let log: String = cv
        .lines
        .iter()
        .skip(start)
        .rev()
        .map(|l| format!("{l}\n"))
        .collect();
    ui.set_carpid_log(log.into());
}

thread_local! {
    static SNAP_TICKS: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
}

/// Index order of the UI's `held` model.
const KEY_ORDER: [&str; 23] = [
    "prev",
    "next",
    "up",
    "down",
    "enter",
    "return",
    "setup",
    "nav",
    "tel",
    "media",
    "name",
    "info",
    "car",
    "w-up",
    "w-down",
    "w-click",
    "w-mode",
    "w-ptt",
    "w-volup",
    "w-voldown",
    "w-volclick",
    "s-up",
    "s-down",
];

fn key_name(k: keys::AnyKey) -> &'static str {
    use keys::{AnyKey, Panel, Stalk, Wheel};
    match k {
        AnyKey::Panel(p) => match p {
            Panel::Prev => "prev",
            Panel::Next => "next",
            Panel::Up => "up",
            Panel::Down => "down",
            Panel::Enter => "enter",
            Panel::Return => "return",
            Panel::Setup => "setup",
            Panel::Nav => "nav",
            Panel::Tel => "tel",
            Panel::Media => "media",
            Panel::Name => "name",
            Panel::Info => "info",
            Panel::Car => "car",
        },
        AnyKey::Wheel(w) => match w {
            Wheel::Up => "w-up",
            Wheel::Down => "w-down",
            Wheel::Click => "w-click",
            Wheel::Mode => "w-mode",
            Wheel::Ptt => "w-ptt",
            Wheel::VolUp => "w-volup",
            Wheel::VolDown => "w-voldown",
            Wheel::VolClick => "w-volclick",
        },
        AnyKey::Stalk(Stalk::Up) => "s-up",
        AnyKey::Stalk(Stalk::Down) => "s-down",
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn livi_config_gets_knob_bindings_and_keeps_its_own() {
        let dir = std::env::temp_dir().join(format!("rnse-sim-seed-{}", std::process::id()));
        let file = dir.join("LIVI/config.json");
        std::fs::create_dir_all(file.parent().unwrap()).unwrap();
        std::fs::write(
            &file,
            r#"{"carName":"TT","bindings":{"knobLeft":"KeyQ","knobRight":"","down":"ArrowDown"}}"#,
        )
        .unwrap();
        super::seed_livi_config(&dir).unwrap();
        let v: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
        assert_eq!(v["carName"], "TT");
        assert_eq!(v["bindings"]["knobLeft"], "KeyQ");
        assert_eq!(v["bindings"]["knobRight"], "PageDown");
        assert_eq!(v["bindings"]["down"], "ArrowDown");
        assert_eq!(v["androidAuto"]["keys"], "rotary");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
