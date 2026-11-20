//! carpid (and, on Linux, LIVI) as child processes: carpid's config in the
//! work folder, its own HOME and state dirs (so a desktop run never touches
//! anything else), and their logs, where carpid's key routing lines
//! (`carpid::act`) become the event list.

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::io::{AsyncBufReadExt, BufReader};

#[derive(Default)]
pub struct CarpidView {
    pub status: String,
    pub lines: VecDeque<String>,
    /// Routing lines, with when they arrived.
    pub routes: VecDeque<(Instant, String)>,
    pub child: Option<tokio::process::Child>,
    /// Bumped on each start, so an old run's exit doesn't report over a new one.
    pub generation: u64,
    pub quit: bool,
}

/// One child process and where its log goes.
pub struct Runner {
    pub label: &'static str,
    pub bin: PathBuf,
    pub args: Vec<std::ffi::OsString>,
    pub env: Vec<(String, std::ffi::OsString)>,
    pub env_remove: Vec<String>,
    pub work: PathBuf,
    /// Type carpid's LIVI keys into LIVI's window from Windows: uinput's
    /// stand-in where the compositor can't see uinput (WSLg).
    pub xkeys: bool,
    pub view: Arc<Mutex<CarpidView>>,
}

const TEMPLATE: &str = include_str!("../carpid-sim.toml");

/// Writes the sim's carpid.toml into `work` unless one is there (keeps
/// edits), and points its `[api] socket` at `socket` either way.
pub fn ensure_config(work: &Path, socket: &str) -> anyhow::Result<PathBuf> {
    let path = work.join("carpid.toml");
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(_) => TEMPLATE.to_string(),
    };
    let mut out = String::new();
    for line in text.lines() {
        if line.trim_start().starts_with("socket =") {
            out.push_str(&format!(
                "socket = \"{socket}\"  # set by rnse-sim at each start\n"
            ));
        } else {
            out.push_str(line);
            out.push('\n');
        }
    }
    std::fs::write(&path, out)?;
    Ok(path)
}

/// `KEY_*` that LIVI's uinput keyboard would type for a LiviKey, for the log
/// (`crates/carpi-input/src/livi.rs`).
fn uinput_name(key: &str) -> Option<&'static str> {
    Some(match key {
        "Up" => "KEY_UP (up)",
        "Down" => "KEY_DOWN (down)",
        "Left" => "KEY_LEFT (left)",
        "Right" => "KEY_RIGHT (right)",
        "Select" => "KEY_ENTER (selectDown)",
        "Back" => "KEY_BACKSPACE (back)",
        "Home" => "KEY_H (home)",
        "PlayPause" => "KEY_P (playPause)",
        "Next" => "KEY_N (next)",
        "Prev" => "KEY_B (prev)",
        "AcceptPhone" => "KEY_A (acceptPhone)",
        "RejectPhone" => "KEY_R (rejectPhone)",
        "Voice" => "KEY_V (voiceAssistant)",
        "CycleSession" => "KEY_S (cycleSession)",
        "KnobLeft" => "KEY_PAGEUP (knobLeft)",
        "KnobRight" => "KEY_PAGEDOWN (knobRight)",
        _ => return None,
    })
}

/// The LiviKey of a `key X -> LIVI` line.
fn livi_key(line: &str) -> Option<&str> {
    let (_, msg) = line.split_once("carpid::act: key ")?;
    let (key, dest) = msg.split_once(" -> ")?;
    dest.starts_with("LIVI").then_some(key)
}

/// LIVI's binding of a LiviKey (`livi-core-proto` config: up=ArrowUp ...
/// knobLeft=PageUp, knobRight=PageDown), as a Windows virtual-key code.
fn vk(key: &str) -> Option<u8> {
    Some(match key {
        "Up" => 0x26,
        "Down" => 0x28,
        "Left" => 0x25,
        "Right" => 0x27,
        "Select" => 0x0D,
        "Back" => 0x08,
        "Home" => b'H',
        "PlayPause" => b'P',
        "Next" => b'N',
        "Prev" => b'B',
        "AcceptPhone" => b'A',
        "RejectPhone" => b'R',
        "Voice" => b'V',
        "CycleSession" => b'S',
        "KnobLeft" => 0x21,
        "KnobRight" => 0x22,
        _ => return None,
    })
}

/// Types into LIVI's window the way a keyboard would. Under WSLg, LIVI's
/// Wayland window is a Windows window and its keys come from Windows (uinput
/// in the WSL kernel reaches no compositor), so one persistent PowerShell
/// (`wsl/livi-keys.ps1`, through WSL interop) focuses the LIVI window, presses
/// the key with its scan code, and gives focus back. Keys go in order.
fn type_key(key: &str) {
    static QUEUE: std::sync::OnceLock<Option<Mutex<std::process::ChildStdin>>> =
        std::sync::OnceLock::new();
    let Some(code) = vk(key) else { return };
    let q = QUEUE.get_or_init(|| {
        let script = crate::repo_root().join("tools/rnse-sim/wsl/livi-keys.ps1");
        let win = std::process::Command::new("wslpath")
            .arg("-w")
            .arg(&script)
            .output()
            .ok()?;
        let win = String::from_utf8_lossy(&win.stdout).trim().to_string();
        let child = std::process::Command::new("powershell.exe")
            .args([
                "-NoProfile",
                "-NonInteractive",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                &win,
            ])
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .ok()?;
        child.stdin.map(Mutex::new)
    });
    if let Some(stdin) = q {
        use std::io::Write;
        let mut s = stdin.lock().unwrap();
        let _ = writeln!(s, "{code}");
        let _ = s.flush();
    }
}

fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut it = s.chars().peekable();
    while let Some(c) = it.next() {
        if c == '\u{1b}' {
            if it.peek() == Some(&'[') {
                it.next();
                for c in it.by_ref() {
                    if c.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
            continue;
        }
        out.push(c);
    }
    out
}

/// A routing event from one carpid log line, if it is one.
fn route_event(line: &str) -> Option<String> {
    let (_, msg) = line.split_once("carpid::act: ")?;
    if let Some(rest) = msg.strip_prefix("key ") {
        // `key KnobRight -> LIVI (LIVI focus livi)`
        let (key, dest) = rest.split_once(" -> ")?;
        if let Some(rest) = dest.strip_prefix("LIVI") {
            let ui = uinput_name(key).unwrap_or("uinput");
            return Some(format!("LIVI ← {key}  ⇒  {ui}  {}", rest.trim()));
        }
        // AA/CarPlay: the command shows up from the bridge; keep the route.
        return Some(format!("carpid: {key} → {dest}"));
    }
    if msg.starts_with("DIS key")
        || msg.starts_with("car action")
        || msg.starts_with("would run")
        || msg.starts_with("capture")
        || msg.starts_with("action")
        || msg.contains("faceplate")
    {
        return Some(format!("carpid: {msg}"));
    }
    None
}

impl Runner {
    pub fn start(&self, rt: &tokio::runtime::Handle) {
        if self.label == "carpid" {
            // The last run was killed, not crashed: drop its marker so carpid
            // doesn't take a crash capture every start.
            let _ = std::fs::remove_file(self.work.join("state/carpi/carpid.running"));
        }
        let mut cmd = tokio::process::Command::new(&self.bin);
        cmd.args(&self.args)
            .envs(self.env.iter().map(|(k, v)| (k, v)))
            .current_dir(&self.work)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        #[cfg(windows)]
        {
            // No console window of its own.
            cmd.creation_flags(0x0800_0000);
        }
        for k in &self.env_remove {
            cmd.env_remove(k);
        }
        let label = self.label;
        let xkeys = self.xkeys;
        let mut view = self.view.lock().unwrap();
        view.generation += 1;
        let generation = view.generation;
        let _guard = rt.enter();
        match cmd.spawn() {
            Err(e) => {
                view.status = format!("can't start {label} ({}): {e}", self.bin.display());
            }
            Ok(mut child) => {
                view.status = format!(
                    "{label} running (pid {}) · work {}",
                    child.id().unwrap_or(0),
                    self.work.display()
                );
                for pipe in [
                    child
                        .stdout
                        .take()
                        .map(|p| Box::new(p) as Box<dyn tokio::io::AsyncRead + Unpin + Send>),
                    child
                        .stderr
                        .take()
                        .map(|p| Box::new(p) as Box<dyn tokio::io::AsyncRead + Unpin + Send>),
                ]
                .into_iter()
                .flatten()
                {
                    let view = self.view.clone();
                    rt.spawn(async move {
                        let mut lines = BufReader::new(pipe).lines();
                        while let Ok(Some(l)) = lines.next_line().await {
                            let l = strip_ansi(&l);
                            let mut v = view.lock().unwrap();
                            if xkeys && let Some(k) = livi_key(&l) {
                                type_key(k);
                            }
                            if let Some(e) = route_event(&l) {
                                v.routes.push_back((Instant::now(), e));
                                while v.routes.len() > 60 {
                                    v.routes.pop_front();
                                }
                            }
                            v.lines.push_back(if label == "carpid" {
                                l
                            } else {
                                format!("[{label}] {l}")
                            });
                            while v.lines.len() > 400 {
                                v.lines.pop_front();
                            }
                        }
                    });
                }
                view.child = Some(child);
                let view = self.view.clone();
                rt.spawn(async move {
                    loop {
                        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
                        let mut v = view.lock().unwrap();
                        if v.generation != generation || v.quit {
                            return;
                        }
                        let Some(child) = v.child.as_mut() else {
                            return;
                        };
                        if let Ok(Some(st)) = child.try_wait() {
                            v.status = format!("{label} exited ({st}); see its log, then Restart");
                            v.child = None;
                            return;
                        }
                    }
                });
            }
        }
    }

    pub fn restart(&self, rt: &tokio::runtime::Handle) {
        stop_all(&self.view);
        self.view
            .lock()
            .unwrap()
            .lines
            .push_back("---- restart ----".into());
        self.start(rt);
    }
}

/// Ends carpid. It has no SIGTERM on Windows and holds no hardware here, so
/// it is killed.
pub fn stop_all(view: &Arc<Mutex<CarpidView>>) {
    let child = view.lock().unwrap().child.take();
    if let Some(mut c) = child {
        let _ = c.start_kill();
        // Give it a moment to go, so the hub port and API port free up.
        for _ in 0..50 {
            if matches!(c.try_wait(), Ok(Some(_))) {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_lines_become_events() {
        let l =
            "2026-11-19T10:00:00.000Z  INFO carpid::act: key KnobRight -> LIVI (LIVI focus livi)";
        assert_eq!(
            route_event(l).as_deref(),
            Some("LIVI ← KnobRight  ⇒  KEY_PAGEDOWN (knobRight)  (LIVI focus livi)")
        );
        let l = "x INFO carpid::act: key Select -> AA/CarPlay (LIVI focus projection)";
        assert_eq!(
            route_event(l).as_deref(),
            Some("carpid: Select → AA/CarPlay (LIVI focus projection)")
        );
        assert_eq!(route_event("x INFO carpid: listening on ican"), None);
        assert_eq!(strip_ansi("\u{1b}[32mINFO\u{1b}[0m x"), "INFO x");
    }
}
