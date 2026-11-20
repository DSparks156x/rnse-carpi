//! The rest of the car on ICAN: NM ring, ignition, the RNS-E's source, and
//! the values carpid and LIVI read (speed, wheels, steering, gear, lights,
//! doors, dimming). Each message goes out at its DBC cycle with the bit
//! layout carpi-signals decodes; the tests check every field round-trips
//! through `carpi_signals::ican::decode`.
//!
//! Bytes the decoders don't read keep the values of the parked car log
//! (`RNS-E-Hudiy/references/canlog.txt`).

use carpi_can::Frame;
use carpi_signals::ids;
use std::time::{Duration, Instant};

/// TT 8J: wheelbase and track, m; steering ratio, wheel deg per road-wheel deg.
const WHEELBASE_M: f64 = 2.468;
const TRACK_M: f64 = 1.57;
const STEERING_RATIO: f64 = 15.5;
/// ABS pulses per wheel revolution (0x359 `pulses_per_rev`, parked log: 43).
const PULSES_PER_REV: f64 = 43.0;
/// The wheel counters run 0-1000.
const PULSE_WRAP: f64 = 1001.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gear {
    P,
    R,
    N,
    D,
    S,
    /// Manual gate (tiptronic) or a manual box.
    M,
}

impl Gear {
    pub const ALL: [Gear; 6] = [Gear::P, Gear::R, Gear::N, Gear::D, Gear::S, Gear::M];

    /// `trans.selector` raw (0x359 bits 60-63).
    fn raw(self) -> u64 {
        match self {
            Gear::P => 8,
            Gear::R => 7,
            Gear::N => 6,
            Gear::D => 5,
            Gear::S => 12,
            Gear::M => 14,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Gear::P => "P",
            Gear::R => "R",
            Gear::N => "N",
            Gear::D => "D",
            Gear::S => "S",
            Gear::M => "M",
        }
    }
}

#[derive(Clone, Debug)]
pub struct CarState {
    /// The ring is up (gateway 0x42B and RNS-E 0x436). Off: the bus goes
    /// silent, as parked and asleep.
    pub awake: bool,
    pub kl15: bool,
    /// RNS-E on its TV source (0x661 byte 3 = 0x37): carpid only routes keys
    /// and draws top lines while it is.
    pub on_tv: bool,
    pub speed_kmh: f64,
    /// Steering wheel, degrees; positive = left (DBC sign bit set = right).
    pub steer_deg: f64,
    pub gear: Gear,
    pub rpm: f64,
    pub coolant_c: f64,
    pub outside_c: f64,
    pub fuel_l: f64,
    pub battery_v: f64,
    pub side_lights: bool,
    pub low_beam: bool,
    pub high_beam: bool,
    pub fog_front: bool,
    pub turn_left: bool,
    pub turn_right: bool,
    pub hazard: bool,
    pub door_fl: bool,
    pub door_fr: bool,
    pub door_rl: bool,
    pub door_rr: bool,
    pub hood: bool,
    pub handbrake: bool,
    /// Instrument lighting switch (Kl58s), 0-100; nonzero reads as night.
    pub dim_switch_pct: f64,
    /// The cluster's light sensor, raw 0-253 (dark .. bright).
    pub light_sensor: f64,
    /// Display units: true = US cluster (mi, °F, 12 h), false = metric.
    pub us_units: bool,
    pub odometer_km: f64,
}

impl Default for CarState {
    fn default() -> Self {
        CarState {
            awake: true,
            kl15: true,
            on_tv: true,
            speed_kmh: 0.0,
            steer_deg: 0.0,
            gear: Gear::P,
            rpm: 800.0,
            coolant_c: 90.0,
            outside_c: 18.0,
            fuel_l: 40.0,
            battery_v: 14.1,
            side_lights: false,
            low_beam: false,
            high_beam: false,
            fog_front: false,
            turn_left: false,
            turn_right: false,
            hazard: false,
            door_fl: false,
            door_fr: false,
            door_rl: false,
            door_rr: false,
            hood: false,
            handbrake: false,
            dim_switch_pct: 0.0,
            light_sensor: 180.0,
            us_units: true,
            odometer_km: 123_456.0,
        }
    }
}

impl CarState {
    /// Night as the car shows it: lights on, Kl58s up, sensor dark.
    pub fn set_night(&mut self, night: bool) {
        self.side_lights = night;
        self.low_beam = night;
        self.dim_switch_pct = if night { 40.0 } else { 0.0 };
        self.light_sensor = if night { 8.0 } else { 180.0 };
    }

    pub fn night(&self) -> bool {
        self.low_beam && self.dim_switch_pct > 0.0
    }

    /// Wheel speeds, km/h, from speed and steering (bicycle model): FL, FR,
    /// RL, RR. Outer wheels run faster in a turn.
    pub fn wheel_kmh(&self) -> [f64; 4] {
        let v = self.speed_kmh;
        let road = (self.steer_deg / STEERING_RATIO).to_radians();
        if road.abs() < 1e-4 {
            return [v; 4];
        }
        // Turn radius at the rear axle centre; positive = turning left.
        let r = WHEELBASE_M / road.tan();
        let half = TRACK_M / 2.0;
        let rear_l = v * (r - half) / r;
        let rear_r = v * (r + half) / r;
        let front = |side: f64| {
            let rr = r + side;
            v * (rr * rr + WHEELBASE_M * WHEELBASE_M).sqrt() / r.abs()
        };
        [front(-half), front(half), rear_l, rear_r]
    }

    pub fn reversing(&self) -> bool {
        self.gear == Gear::R
    }
}

/// Bit setter, Intel order like `carpi_signals::bits::get`.
fn put(d: &mut [u8], start: u32, len: u32, raw: u64) {
    for i in 0..len {
        let n = start + i;
        let (byte, bit) = ((n / 8) as usize, n % 8);
        if raw >> i & 1 == 1 {
            d[byte] |= 1 << bit;
        } else {
            d[byte] &= !(1 << bit);
        }
    }
}

fn flag(d: &mut [u8], n: u32, on: bool) {
    put(d, n, 1, on as u64);
}

fn scaled(v: f64, factor: f64, offset: f64, max: u64) -> u64 {
    (((v - offset) / factor).round().max(0.0) as u64).min(max)
}

fn frame(id: u32, d: &[u8]) -> Frame {
    Frame::new(id, d).expect("standard id, <= 8 bytes")
}

/// One periodic message: what it is and when it's next due.
struct Slot {
    id: u32,
    cycle: Duration,
    next: Instant,
}

/// Turns [`CarState`] into frames at their cycle times.
pub struct CarBus {
    slots: Vec<Slot>,
    /// Wheel pulse counters (FL, FR, RL, RR) and the front axle's.
    pulses: [f64; 4],
    axle_pulses: f64,
    last_pulse_at: Option<Instant>,
    /// Stalk rocker bits for 0x2C1 byte 2 (0x20 up, 0x10 down).
    pub stalk_bits: u8,
    /// Steering wheel command byte for 0x5C3 (00 = none).
    pub wheel_cmd: u8,
}

/// Cycle for an id: the decoder table's, else ours.
fn cycle(id: u32) -> Duration {
    match id {
        ids::NM_BASE..=ids::NM_LAST => Duration::from_millis(93),
        ids::RADIO_4 => Duration::from_millis(100),
        ids::STALK_KEYS => Duration::from_millis(100),
        // Wheel keys: ~500 ms (decoder note), plus a frame on each change.
        ids::MFL_CMD => Duration::from_millis(500),
        _ => carpi_signals::ican::message(id)
            .map(|m| m.cycle)
            .unwrap_or(Duration::from_millis(100)),
    }
}

impl CarBus {
    pub fn new(now: Instant) -> Self {
        let list = [
            ids::NM_BASE + u32::from(ids::NM_NODE_GATEWAY),
            ids::NM_BASE + u32::from(ids::NM_NODE_RNSE),
            ids::ZAS_STATUS,
            ids::BSG_3,
            ids::BSG_2,
            ids::RADIO_4,
            ids::GW_BREMSE_GETRIEBE,
            ids::GATEWAY_1,
            ids::GW_MOTOR,
            ids::LENKWINKEL_1,
            ids::BSG_KOMBI,
            ids::GW_KOMBI,
            ids::BREMSE_10,
            ids::KOMBI_K1,
            ids::DIMMUNG,
            ids::DIAGNOSE_1,
            ids::EINHEITEN,
            ids::STALK_KEYS,
            ids::MFL_CMD,
        ];
        let slots = list
            .iter()
            .enumerate()
            .map(|(i, &id)| Slot {
                id,
                cycle: cycle(id),
                // Spread the first round so they don't all land at once.
                next: now + Duration::from_millis(i as u64 * 3),
            })
            .collect();
        CarBus {
            slots,
            pulses: [0.0; 4],
            axle_pulses: 0.0,
            last_pulse_at: None,
            stalk_bits: 0,
            wheel_cmd: 0,
        }
    }

    /// Frames due by `now`.
    pub fn poll(&mut self, car: &CarState, now: Instant, out: &mut Vec<Frame>) {
        self.count_pulses(car, now);
        if !car.awake {
            return;
        }
        for i in 0..self.slots.len() {
            if self.slots[i].next > now {
                continue;
            }
            let s = &mut self.slots[i];
            s.next = (s.next + s.cycle).max(now);
            let id = s.id;
            if let Some(f) = self.build(car, id) {
                out.push(f);
            }
        }
    }

    /// Sends `id` now (a key frame that changed), and restarts its cycle.
    pub fn send_now(&mut self, car: &CarState, id: u32, now: Instant) -> Option<Frame> {
        if !car.awake {
            return None;
        }
        if let Some(s) = self.slots.iter_mut().find(|s| s.id == id) {
            s.next = now + s.cycle;
        }
        self.build(car, id)
    }

    fn count_pulses(&mut self, car: &CarState, now: Instant) {
        let dt = self
            .last_pulse_at
            .map_or(0.0, |t| now.saturating_duration_since(t).as_secs_f64());
        self.last_pulse_at = Some(now);
        let circ_m = 2.004;
        for (p, kmh) in self.pulses.iter_mut().zip(car.wheel_kmh()) {
            *p = (*p + kmh / 3.6 / circ_m * PULSES_PER_REV * dt).rem_euclid(PULSE_WRAP);
        }
        // The front axle counter is 11 bits, wrapping at 2048.
        let front = (car.wheel_kmh()[0] + car.wheel_kmh()[1]) / 2.0;
        self.axle_pulses =
            (self.axle_pulses + front / 3.6 / circ_m * PULSES_PER_REV * dt).rem_euclid(2048.0);
    }

    fn build(&self, c: &CarState, id: u32) -> Option<Frame> {
        let gw = ids::NM_BASE + u32::from(ids::NM_NODE_GATEWAY);
        let rnse = ids::NM_BASE + u32::from(ids::NM_NODE_RNSE);
        Some(match id {
            // Ring: each names the other as successor, ring + alive bits.
            _ if id == gw => frame(id, &[ids::NM_NODE_RNSE, 0x01, 0, 0, 0, 0]),
            _ if id == rnse => frame(id, &[ids::NM_NODE_GATEWAY, 0x01, 0, 0, 0, 0]),
            ids::ZAS_STATUS => {
                // S, Kl15, X with the key on; S alone with it off.
                let b = if c.kl15 { 0x07 } else { 0x01 };
                frame(id, &[b])
            }
            ids::BSG_3 => {
                let mut d = [0xC7, 0x02, 0x00, 0x80];
                flag(&mut d, 1, c.kl15);
                flag(&mut d, 2, c.kl15);
                frame(id, &d)
            }
            ids::BSG_2 => {
                let mut d = [0x89, 0x5F, 0x00, 0x0B, 0x04, 0x00];
                put(&mut d, 0, 8, scaled(c.battery_v, 0.05, 5.0, 254));
                frame(id, &d)
            }
            ids::RADIO_4 => {
                let src = if c.on_tv {
                    carpi_signals::ican::RADIO_4_TV
                } else {
                    0x01
                };
                frame(id, &[0x83, 0x01, 0x12, src, 0, 0, 0, 0])
            }
            ids::GW_BREMSE_GETRIEBE => {
                let mut d = [0x20, 0x01, 0x00, 0x00, 0x00, 0x6B, 0x40, 0x88];
                // Qualifier bits 0-5 clear: everything valid.
                put(&mut d, 0, 6, 0);
                put(&mut d, 9, 15, scaled(c.speed_kmh, 0.01, 0.0, 32639));
                put(&mut d, 24, 11, self.axle_pulses as u64);
                put(&mut d, 40, 6, PULSES_PER_REV as u64);
                flag(&mut d, 51, false);
                put(&mut d, 60, 4, c.gear.raw());
                frame(id, &d)
            }
            ids::GATEWAY_1 => {
                let mut d = [0u8; 8];
                put(&mut d, 9, 15, scaled(c.speed_kmh, 0.01, 0.0, 32639));
                flag(&mut d, 1, c.reversing());
                frame(id, &d)
            }
            ids::GW_MOTOR => {
                let mut d = [0x08, 0x00, 0x00, 0x50, 0x20, 0x18, 0x00, 0x00];
                put(&mut d, 0, 4, 0);
                let rpm = if c.kl15 { c.rpm } else { 0.0 };
                put(&mut d, 8, 16, scaled(rpm, 0.25, 0.0, 0xFEFF));
                put(&mut d, 24, 8, scaled(c.coolant_c, 0.75, -48.0, 254).max(1));
                flag(&mut d, 34, false);
                frame(id, &d)
            }
            ids::LENKWINKEL_1 => {
                let mut d = [0x22, 0x00, 0x00, 0x00, 0x80, 0xE0, 0x79, 0xFD];
                put(
                    &mut d,
                    0,
                    15,
                    scaled(c.steer_deg.abs(), 0.04375, 0.0, 0x7FFF),
                );
                // Sign bit set = negative = right.
                flag(&mut d, 15, c.steer_deg < 0.0);
                put(&mut d, 16, 15, 0);
                flag(&mut d, 31, false);
                put(&mut d, 41, 2, 0);
                frame(id, &d)
            }
            ids::BSG_KOMBI => {
                let mut d = [0x80, 0x00, 0x01, 0x3A, 0x20, 0x00, 0x00, 0x00];
                // Indicators blink at ~1.5 Hz.
                let blink = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|t| t.as_millis() % 666 < 333)
                    .unwrap_or(true);
                flag(&mut d, 0, (c.turn_left || c.hazard) && blink);
                flag(&mut d, 1, (c.turn_right || c.hazard) && blink);
                flag(&mut d, 3, c.hazard);
                flag(&mut d, 5, c.reversing());
                flag(&mut d, 8, c.door_fl);
                flag(&mut d, 9, c.door_fr);
                flag(&mut d, 10, c.door_rl);
                flag(&mut d, 11, c.door_rr);
                flag(&mut d, 12, c.hood);
                let disp = if c.night() { 40.0 } else { 100.0 };
                put(&mut d, 16, 7, scaled(disp, 1.0, 0.0, 100));
                flag(&mut d, 23, false);
                put(&mut d, 24, 7, scaled(c.dim_switch_pct, 1.0, 0.0, 100));
                flag(&mut d, 31, false);
                flag(&mut d, 40, c.side_lights || c.low_beam);
                flag(&mut d, 43, c.low_beam);
                flag(&mut d, 44, c.fog_front);
                flag(&mut d, 62, c.high_beam);
                flag(&mut d, 63, !c.low_beam && c.kl15);
                frame(id, &d)
            }
            ids::GW_KOMBI => {
                let mut d = [0x10, 0x01, 0x00, 0x40, 0x7D, 0x8B, 0x8B, 0x00];
                put(&mut d, 40, 8, scaled(c.outside_c, 0.5, -50.0, 254));
                flag(&mut d, 2, false);
                put(&mut d, 48, 8, scaled(c.outside_c, 0.5, -50.0, 254).max(2));
                frame(id, &d)
            }
            ids::BREMSE_10 => {
                let mut d = [0x02, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                for (i, p) in self.pulses.iter().enumerate() {
                    let i = i as u32;
                    put(&mut d, 16 + 10 * i, 10, (*p as u64).min(1000));
                    flag(&mut d, 12 + i, false);
                    flag(&mut d, 56 + i, false);
                    flag(&mut d, 60 + i, c.reversing() && c.speed_kmh > 0.0);
                }
                frame(id, &d)
            }
            ids::KOMBI_K1 => {
                let mut d = [0x20, 0x04, 0x24, 0x0B, 0x01, 0x25, 0x23];
                flag(&mut d, 1, c.fuel_l < 7.0);
                flag(&mut d, 5, c.handbrake);
                put(&mut d, 24, 7, scaled(c.fuel_l, 1.0, 0.0, 126));
                let disp = if c.night() { 40.0 } else { 100.0 };
                put(&mut d, 40, 7, scaled(disp, 1.0, 0.0, 100));
                put(&mut d, 48, 8, scaled(c.light_sensor, 1.0, 0.0, 253));
                frame(id, &d)
            }
            ids::DIMMUNG => {
                let mut d = [0x25, 0x00, 0x23];
                let disp = if c.night() { 40.0 } else { 100.0 };
                put(&mut d, 0, 7, scaled(disp, 1.0, 0.0, 100));
                flag(&mut d, 7, false);
                put(&mut d, 8, 7, scaled(c.dim_switch_pct, 1.0, 0.0, 100));
                flag(&mut d, 15, false);
                put(&mut d, 16, 8, scaled(c.light_sensor, 1.0, 0.0, 253));
                frame(id, &d)
            }
            ids::DIAGNOSE_1 => {
                let mut d = [0x54, 0x1B, 0xA4, 0xA3, 0x19, 0xDE, 0x6E, 0x14];
                put(&mut d, 8, 20, c.odometer_km as u64);
                let (y, mo, da, h, mi, s) = local_now();
                put(&mut d, 28, 7, y.saturating_sub(2000));
                put(&mut d, 35, 4, mo);
                put(&mut d, 39, 5, da);
                put(&mut d, 44, 5, h);
                put(&mut d, 49, 6, mi);
                put(&mut d, 55, 6, s);
                flag(&mut d, 62, false);
                flag(&mut d, 63, false);
                frame(id, &d)
            }
            ids::EINHEITEN => {
                let b = if c.us_units { 0xC7 } else { 0x00 };
                frame(id, &[b, 0x00])
            }
            ids::STALK_KEYS => {
                let b2 = 0x80 | (self.stalk_bits & 0x30);
                frame(id, &[0x00, 0x00, b2, 0x00, 0x02])
            }
            ids::MFL_CMD => frame(id, &[0x39, self.wheel_cmd]),
            _ => return None,
        })
    }
}

/// The PC clock as local-ish civil time (UTC; carpid only shows it).
fn local_now() -> (u64, u64, u64, u64, u64, u64) {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let days = secs / 86_400;
    let rem = secs % 86_400;
    // Civil from days (Howard Hinnant's algorithm).
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    (
        y as u64,
        m as u64,
        d as u64,
        rem / 3600,
        rem % 3600 / 60,
        rem % 60,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use carpi_signals::ican::decode;
    use std::collections::HashMap;

    use carpi_state::Value;

    type Values = HashMap<&'static str, Option<Value>>;

    fn values(f: &Frame) -> Values {
        let mut out = HashMap::new();
        decode(f.id, f.data(), &mut |name, v| {
            out.insert(name, v);
        });
        out
    }

    fn num(v: &Values, name: &str) -> f64 {
        match v.get(name) {
            Some(Some(Value::Num(n))) => *n,
            other => panic!("{name}: {other:?}"),
        }
    }

    fn boolean(v: &Values, name: &str) -> bool {
        match v.get(name) {
            Some(Some(Value::Bool(b))) => *b,
            other => panic!("{name}: {other:?}"),
        }
    }

    fn built(c: &CarState, id: u32) -> Values {
        let bus = CarBus::new(Instant::now());
        values(&bus.build(c, id).expect("built"))
    }

    #[test]
    fn speed_gear_and_steering_decode() {
        let c = CarState {
            speed_kmh: 87.5,
            gear: Gear::R,
            steer_deg: -123.4,
            ..Default::default()
        };
        let v = built(&c, ids::GW_BREMSE_GETRIEBE);
        assert!((num(&v, "vehicle.speed_kmh") - 87.5).abs() < 0.01);
        assert_eq!(num(&v, "trans.selector"), 7.0);
        assert_eq!(num(&v, "vehicle.pulses_per_rev"), 43.0);
        let v = built(&c, ids::LENKWINKEL_1);
        assert!((num(&v, "steering.angle_deg") + 123.4).abs() < 0.05);
        let v = built(&c, ids::GATEWAY_1);
        assert!((num(&v, "vehicle.speed_gw_kmh") - 87.5).abs() < 0.01);
    }

    #[test]
    fn lights_doors_and_night() {
        let mut c = CarState {
            door_fl: true,
            hood: true,
            high_beam: true,
            ..Default::default()
        };
        c.set_night(true);
        let v = built(&c, ids::BSG_KOMBI);
        assert!(boolean(&v, "body.door_fl_open"));
        assert!(!boolean(&v, "body.door_rr_open"));
        assert!(boolean(&v, "body.hood_open"));
        assert!(boolean(&v, "lights.low_beam"));
        assert!(boolean(&v, "lights.high_beam"));
        assert_eq!(num(&v, "light.kl58t_pct"), 40.0);
        let v = built(&c, ids::DIMMUNG);
        assert_eq!(num(&v, "light.dim_switch_pct"), 40.0);
        assert_eq!(num(&v, "light.dim_sensor"), 8.0);
    }

    #[test]
    fn ignition_engine_and_clock() {
        let c = CarState::default();
        let v = built(&c, ids::BSG_3);
        assert!(boolean(&v, "ignition.kl15"));
        let v = built(&c, ids::ZAS_STATUS);
        assert!(boolean(&v, "ignition.zas.kl15"));
        let v = built(&c, ids::GW_MOTOR);
        assert_eq!(num(&v, "engine.rpm"), 800.0);
        assert!((num(&v, "engine.coolant_c") - 90.0).abs() < 0.5);
        let v = built(&c, ids::DIAGNOSE_1);
        assert_eq!(num(&v, "vehicle.odometer_km"), 123_456.0);
        assert!(matches!(
            v.get("clock.datetime"),
            Some(Some(Value::Text(_)))
        ));
        let v = built(&c, ids::RADIO_4);
        assert!(boolean(&v, "rnse.on_tv"));
        let off = CarState {
            kl15: false,
            ..Default::default()
        };
        assert!(!boolean(&built(&off, ids::BSG_3), "ignition.kl15"));
        assert!(!boolean(&built(&off, ids::ZAS_STATUS), "ignition.zas.kl15"));
    }

    #[test]
    fn wheel_pulses_advance_and_split_in_a_turn() {
        let t0 = Instant::now();
        let c = CarState {
            speed_kmh: 36.0,
            steer_deg: 180.0,
            ..Default::default()
        };
        let w = c.wheel_kmh();
        // Turning left: right side faster than left.
        assert!(w[1] > w[0] && w[3] > w[2], "{w:?}");
        let mut bus = CarBus::new(t0);
        bus.count_pulses(&c, t0);
        bus.count_pulses(&c, t0 + Duration::from_millis(500));
        let v = values(&bus.build(&c, ids::BREMSE_10).unwrap());
        // 10 m/s / 2.004 m * 43 * 0.5 s ~ 107 pulses, a bit less on the inside.
        let fl = num(&v, "wheels.fl_pulses");
        let rr = num(&v, "wheels.rr_pulses");
        assert!((90.0..125.0).contains(&fl), "{fl}");
        assert!(rr > num(&v, "wheels.rl_pulses"));
        assert!(!boolean(&v, "wheels.fl_reverse"));
    }

    #[test]
    fn asleep_sends_nothing() {
        let t0 = Instant::now();
        let mut bus = CarBus::new(t0);
        let c = CarState {
            awake: false,
            ..Default::default()
        };
        let mut out = Vec::new();
        bus.poll(&c, t0 + Duration::from_secs(1), &mut out);
        assert!(out.is_empty());
        let c = CarState::default();
        bus.poll(&c, t0 + Duration::from_secs(1), &mut out);
        assert!(out.iter().any(|f| f.id == 0x42B));
        assert!(out.iter().any(|f| f.id == 0x436));
    }
}
