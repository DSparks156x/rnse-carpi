//! The car's keys as the frames they really send, with the real timing:
//!
//! - RNS-E faceplate (0x461 `37 30 SS K1 K2 00`): `SS` 01 on press, repeated
//!   every 110 ms while held; 04 on release, five times. The knob sends one
//!   press frame per detent (`00 20` clockwise, `00 40` counter-clockwise), so
//!   its speed is the frame rate; a release burst follows when it stops.
//! - Steering wheel (0x5C3 `39 CC`): the command while held, `00` otherwise,
//!   every 500 ms and on each change.
//! - Stalk rocker (0x2C1 byte 2, 0x20 up / 0x10 down), every 100 ms.
//!
//! Codes and timings are carpi-input's (`decode.rs`); nothing here decides
//! what a key means, carpid does.

use crate::car::{CarBus, CarState};
use carpi_can::Frame;
use carpi_signals::ids;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

pub const PANEL_REPEAT: Duration = Duration::from_millis(110);
const RELEASES: usize = 5;
/// The knob counts as stopped after this long without a detent.
const KNOB_IDLE: Duration = Duration::from_millis(200);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Panel {
    Prev,
    Next,
    Up,
    Down,
    Enter,
    Return,
    Setup,
    Nav,
    Tel,
    Media,
    Name,
    Info,
    Car,
}

impl Panel {
    pub fn code(self) -> (u8, u8) {
        match self {
            Panel::Prev => (0x01, 0x00),
            Panel::Next => (0x02, 0x00),
            Panel::Up => (0x40, 0x00),
            Panel::Down => (0x80, 0x00),
            Panel::Enter => (0x00, 0x10),
            Panel::Return => (0x00, 0x02),
            Panel::Setup => (0x00, 0x01),
            Panel::Nav => (0x08, 0x00),
            Panel::Tel => (0x00, 0x08),
            Panel::Media => (0x00, 0x04),
            Panel::Name => (0x04, 0x00),
            Panel::Info => (0x00, 0x0C),
            Panel::Car => (0x0C, 0x00),
        }
    }

    pub fn from_name(s: &str) -> Option<Panel> {
        Some(match s {
            "prev" => Panel::Prev,
            "next" => Panel::Next,
            "up" => Panel::Up,
            "down" => Panel::Down,
            "enter" => Panel::Enter,
            "return" => Panel::Return,
            "setup" => Panel::Setup,
            "nav" => Panel::Nav,
            "tel" => Panel::Tel,
            "media" => Panel::Media,
            "name" => Panel::Name,
            "info" => Panel::Info,
            "car" => Panel::Car,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wheel {
    Up,
    Down,
    Click,
    Mode,
    Ptt,
    VolUp,
    VolDown,
    VolClick,
}

impl Wheel {
    pub fn code(self) -> u8 {
        match self {
            Wheel::Up => 0x0B,
            Wheel::Down => 0x0C,
            Wheel::Click => 0x08,
            Wheel::Mode => 0x1C,
            Wheel::Ptt => 0x2A,
            Wheel::VolUp => 0x06,
            Wheel::VolDown => 0x07,
            Wheel::VolClick => 0xA7,
        }
    }

    pub fn from_name(s: &str) -> Option<Wheel> {
        Some(match s {
            "w-up" => Wheel::Up,
            "w-down" => Wheel::Down,
            "w-click" => Wheel::Click,
            "w-mode" => Wheel::Mode,
            "w-ptt" => Wheel::Ptt,
            "w-volup" => Wheel::VolUp,
            "w-voldown" => Wheel::VolDown,
            "w-volclick" => Wheel::VolClick,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stalk {
    Up,
    Down,
}

impl Stalk {
    fn bit(self) -> u8 {
        match self {
            Stalk::Up => 0x20,
            Stalk::Down => 0x10,
        }
    }

    pub fn from_name(s: &str) -> Option<Stalk> {
        match s {
            "s-up" => Some(Stalk::Up),
            "s-down" => Some(Stalk::Down),
            _ => None,
        }
    }
}

/// Any key, by the name the UI uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnyKey {
    Panel(Panel),
    Wheel(Wheel),
    Stalk(Stalk),
}

impl AnyKey {
    pub fn from_name(s: &str) -> Option<AnyKey> {
        Panel::from_name(s)
            .map(AnyKey::Panel)
            .or_else(|| Wheel::from_name(s).map(AnyKey::Wheel))
            .or_else(|| Stalk::from_name(s).map(AnyKey::Stalk))
    }
}

fn is_button_release(f: &Frame) -> bool {
    let d = f.data();
    f.id == ids::RNSE_KEYS
        && d.len() >= 5
        && d[2] == 0x04
        && !matches!((d[3], d[4]), (0x00, 0x20) | (0x00, 0x40))
}

fn panel_frame(ss: u8, (k1, k2): (u8, u8)) -> Frame {
    Frame::new(ids::RNSE_KEYS, &[0x37, 0x30, ss, k1, k2, 0x00]).expect("6 bytes")
}

pub struct Keys {
    /// Faceplate key held, and when its next repeat is due.
    panel: Option<(Panel, Instant)>,
    wheel: Option<Wheel>,
    stalk: Option<Stalk>,
    /// Frames queued for a time: releases, knob detents.
    queued: VecDeque<(Instant, Frame)>,
    /// Scripted releases (a tap of a given length).
    ups: Vec<(Instant, AnyKey)>,
    /// When the knob may send its next detent, and the last direction.
    knob_free_at: Instant,
    knob_last: Option<(Instant, (u8, u8))>,
    /// Press frames per detent (carpid's `knob_frames_per_detent` is 1).
    pub knob_frames_per_detent: u8,
    /// Detents sent, for the UI's rate readout: (time, +1/-1).
    pub detents: VecDeque<(Instant, i8)>,
    /// Net detents turned since start, for drawing the knob.
    pub position: i32,
}

impl Keys {
    pub fn new(now: Instant) -> Self {
        Keys {
            panel: None,
            wheel: None,
            stalk: None,
            queued: VecDeque::new(),
            ups: Vec::new(),
            knob_free_at: now,
            knob_last: None,
            knob_frames_per_detent: 1,
            detents: VecDeque::new(),
            position: 0,
        }
    }

    pub fn held(&self) -> Vec<AnyKey> {
        let mut v = Vec::new();
        v.extend(self.panel.map(|(p, _)| AnyKey::Panel(p)));
        v.extend(self.wheel.map(AnyKey::Wheel));
        v.extend(self.stalk.map(AnyKey::Stalk));
        v
    }

    pub fn down(&mut self, key: AnyKey, now: Instant, bus: &mut CarBus, car: &CarState) {
        match key {
            AnyKey::Panel(p) => {
                if self.panel.is_some_and(|(h, _)| h == p) {
                    return;
                }
                if let Some((old, _)) = self.panel.take() {
                    self.release_panel(old, now);
                }
                // A new press ends the last key's release burst, else its
                // late 04s would release this one (guess: the capture only
                // has isolated presses).
                self.queued.retain(|(_, f)| !is_button_release(f));
                self.queued.push_back((now, panel_frame(0x01, p.code())));
                self.panel = Some((p, now + PANEL_REPEAT));
            }
            AnyKey::Wheel(w) => {
                self.wheel = Some(w);
                bus.wheel_cmd = w.code();
                self.queue_now(bus.send_now(car, ids::MFL_CMD, now), now);
            }
            AnyKey::Stalk(s) => {
                self.stalk = Some(s);
                bus.stalk_bits = s.bit();
                self.queue_now(bus.send_now(car, ids::STALK_KEYS, now), now);
            }
        }
    }

    pub fn up(&mut self, key: AnyKey, now: Instant, bus: &mut CarBus, car: &CarState) {
        match key {
            AnyKey::Panel(p) => {
                if self.panel.is_some_and(|(h, _)| h == p) {
                    self.panel = None;
                    self.release_panel(p, now);
                }
            }
            AnyKey::Wheel(w) => {
                if self.wheel == Some(w) {
                    self.wheel = None;
                    bus.wheel_cmd = 0;
                    self.queue_now(bus.send_now(car, ids::MFL_CMD, now), now);
                }
            }
            AnyKey::Stalk(s) => {
                if self.stalk == Some(s) {
                    self.stalk = None;
                    bus.stalk_bits = 0;
                    self.queue_now(bus.send_now(car, ids::STALK_KEYS, now), now);
                }
            }
        }
    }

    /// Press now, release after `hold`.
    pub fn tap(
        &mut self,
        key: AnyKey,
        hold: Duration,
        now: Instant,
        bus: &mut CarBus,
        car: &CarState,
    ) {
        self.down(key, now, bus, car);
        self.ups.retain(|(_, k)| *k != key);
        self.ups.push((now + hold, key));
    }

    /// Queue `steps` detents (positive = clockwise) at `rate` detents/s,
    /// after any already queued.
    pub fn knob(&mut self, steps: i32, rate: f64, now: Instant) {
        let gap = Duration::from_secs_f64(1.0 / rate.clamp(0.5, 200.0));
        let code = if steps > 0 {
            (0x00, 0x20)
        } else {
            (0x00, 0x40)
        };
        self.position += steps;
        let mut at = self.knob_free_at.max(now);
        for _ in 0..steps.unsigned_abs() {
            for _ in 0..self.knob_frames_per_detent.max(1) {
                self.queued.push_back((at, panel_frame(0x01, code)));
            }
            self.detents.push_back((at, steps.signum() as i8));
            self.knob_last = Some((at, code));
            at += gap;
        }
        self.knob_free_at = at;
    }

    fn release_panel(&mut self, p: Panel, now: Instant) {
        for i in 0..RELEASES {
            self.queued
                .push_back((now + PANEL_REPEAT * i as u32, panel_frame(0x04, p.code())));
        }
    }

    fn queue_now(&mut self, f: Option<Frame>, now: Instant) {
        if let Some(f) = f {
            self.queued.push_back((now, f));
        }
    }

    /// Frames due by `now`, in time order.
    pub fn poll(&mut self, now: Instant, bus: &mut CarBus, car: &CarState, out: &mut Vec<Frame>) {
        let due: Vec<AnyKey> = self
            .ups
            .iter()
            .filter(|(t, _)| *t <= now)
            .map(|(_, k)| *k)
            .collect();
        self.ups.retain(|(t, _)| *t > now);
        for k in due {
            self.up(k, now, bus, car);
        }
        if let Some((p, next)) = self.panel
            && next <= now
        {
            self.queued.push_back((now, panel_frame(0x01, p.code())));
            self.panel = Some((p, next + PANEL_REPEAT));
        }
        if let Some((last, code)) = self.knob_last
            && now >= last + KNOB_IDLE
        {
            self.knob_last = None;
            for i in 0..RELEASES {
                self.queued.push_back((
                    now + Duration::from_millis(20) * i as u32,
                    panel_frame(0x04, code),
                ));
            }
        }
        // Stable by time; the queue is near-sorted.
        let mut ready: Vec<(Instant, Frame)> = Vec::new();
        self.queued.retain(|(t, f)| {
            if *t <= now {
                ready.push((*t, *f));
                false
            } else {
                true
            }
        });
        ready.sort_by_key(|(t, _)| *t);
        out.extend(ready.into_iter().map(|(_, f)| f));
        while self
            .detents
            .front()
            .is_some_and(|(t, _)| now.saturating_duration_since(*t) > Duration::from_secs(2))
        {
            self.detents.pop_front();
        }
    }

    /// Detents in the last second, signed sum and count, as already sent.
    pub fn knob_rate(&self, now: Instant) -> (i32, usize) {
        let recent = self.detents.iter().filter(|(t, _)| {
            *t <= now && now.saturating_duration_since(*t) <= Duration::from_secs(1)
        });
        recent.fold((0, 0), |(s, n), (_, d)| (s + i32::from(*d), n + 1))
    }
}

/// Mouse wheel over the knob to detents: one notch, one detent, wheel up =
/// clockwise. Slint reports a notch as 60 px (winit `LineDelta` x 60);
/// touchpad pixels add up to the same. A pause drops a part-turn.
#[derive(Default)]
pub struct ScrollWheel {
    acc: f32,
    last: Option<Instant>,
}

impl ScrollWheel {
    pub const NOTCH_PX: f32 = 60.0;

    pub fn feed(&mut self, dy: f32, now: Instant) -> i32 {
        if self
            .last
            .is_none_or(|t| now.saturating_duration_since(t) > Duration::from_millis(400))
        {
            self.acc = 0.0;
        }
        self.last = Some(now);
        self.acc += dy;
        let steps = (self.acc / Self::NOTCH_PX).trunc();
        self.acc -= steps * Self::NOTCH_PX;
        steps as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use carpi_input_check::*;

    /// A copy of the decoder's view, so the frames are checked against what
    /// carpid would make of them without depending on carpi-input here.
    mod carpi_input_check {
        pub fn is_press(d: &[u8]) -> bool {
            d.len() == 6 && d[0] == 0x37 && d[1] == 0x30 && d[2] == 0x01
        }
        pub fn is_release(d: &[u8]) -> bool {
            d.len() == 6 && d[0] == 0x37 && d[1] == 0x30 && d[2] == 0x04
        }
    }

    fn run(
        keys: &mut Keys,
        bus: &mut CarBus,
        car: &CarState,
        t0: Instant,
        ms: u64,
    ) -> Vec<(u64, Frame)> {
        let mut out = Vec::new();
        for t in (0..=ms).step_by(5) {
            let mut v = Vec::new();
            keys.poll(t0 + Duration::from_millis(t), bus, car, &mut v);
            out.extend(v.into_iter().map(|f| (t, f)));
        }
        out
    }

    #[test]
    fn one_wheel_notch_is_one_detent() {
        let t0 = Instant::now();
        let mut w = ScrollWheel::default();
        assert_eq!(w.feed(60.0, t0), 1);
        assert_eq!(w.feed(60.0, t0 + Duration::from_millis(30)), 1);
        assert_eq!(w.feed(-60.0, t0 + Duration::from_millis(60)), -1);
        assert_eq!(w.feed(180.0, t0 + Duration::from_millis(90)), 3);
        // Touchpad: small deltas add up to a notch.
        assert_eq!(w.feed(25.0, t0 + Duration::from_secs(1)), 0);
        assert_eq!(w.feed(25.0, t0 + Duration::from_millis(1010)), 0);
        assert_eq!(w.feed(25.0, t0 + Duration::from_millis(1020)), 1);
        // A pause drops the part-turn.
        assert_eq!(w.feed(50.0, t0 + Duration::from_secs(3)), 0);
        assert_eq!(w.feed(20.0, t0 + Duration::from_secs(4)), 0);
    }

    #[test]
    fn long_next_repeats_then_releases_five_times() {
        let t0 = Instant::now();
        let car = CarState::default();
        let mut bus = CarBus::new(t0);
        let mut k = Keys::new(t0);
        k.tap(
            AnyKey::Panel(Panel::Next),
            Duration::from_millis(1000),
            t0,
            &mut bus,
            &car,
        );
        let got = run(&mut k, &mut bus, &car, t0, 2000);
        let presses: Vec<u64> = got
            .iter()
            .filter(|(_, f)| f.id == 0x461 && is_press(f.data()))
            .map(|(t, _)| *t)
            .collect();
        // 0, 110, ... 990: ten presses over the second.
        assert_eq!(presses.len(), 10, "{presses:?}");
        assert_eq!(presses[0], 0);
        assert!(
            presses
                .windows(2)
                .all(|w| (105..=115).contains(&(w[1] - w[0])))
        );
        let releases = got
            .iter()
            .filter(|(_, f)| f.id == 0x461 && is_release(f.data()))
            .count();
        assert_eq!(releases, 5);
        assert!(
            got.iter()
                .all(|(_, f)| f.id != 0x461 || f.data()[3] == 0x02)
        );
    }

    #[test]
    fn knob_detents_at_rate_then_release() {
        let t0 = Instant::now();
        let car = CarState::default();
        let mut bus = CarBus::new(t0);
        let mut k = Keys::new(t0);
        k.knob(4, 20.0, t0);
        k.knob(-1, 20.0, t0);
        let got = run(&mut k, &mut bus, &car, t0, 1000);
        let cw: Vec<u64> = got
            .iter()
            .filter(|(_, f)| is_press(f.data()) && f.data()[4] == 0x20)
            .map(|(t, _)| *t)
            .collect();
        assert_eq!(cw, vec![0, 50, 100, 150]);
        let ccw = got
            .iter()
            .filter(|(_, f)| is_press(f.data()) && f.data()[4] == 0x40)
            .count();
        assert_eq!(ccw, 1);
        assert!(got.iter().any(|(_, f)| is_release(f.data())));
        assert_eq!(k.knob_rate(t0 + Duration::from_millis(300)), (3, 5));
    }

    #[test]
    fn wheel_and_stalk_send_on_change() {
        let t0 = Instant::now();
        let car = CarState::default();
        let mut bus = CarBus::new(t0);
        let mut k = Keys::new(t0);
        k.tap(
            AnyKey::Wheel(Wheel::Click),
            Duration::from_millis(300),
            t0,
            &mut bus,
            &car,
        );
        k.tap(
            AnyKey::Stalk(Stalk::Up),
            Duration::from_millis(300),
            t0,
            &mut bus,
            &car,
        );
        let got = run(&mut k, &mut bus, &car, t0, 400);
        let wheel: Vec<(u64, u8)> = got
            .iter()
            .filter(|(_, f)| f.id == 0x5C3)
            .map(|(t, f)| (*t, f.data()[1]))
            .collect();
        assert_eq!(wheel, vec![(0, 0x08), (300, 0x00)]);
        let stalk: Vec<(u64, u8)> = got
            .iter()
            .filter(|(_, f)| f.id == 0x2C1)
            .map(|(t, f)| (*t, f.data()[2]))
            .collect();
        assert_eq!(stalk, vec![(0, 0xA0), (300, 0x80)]);
    }
}
