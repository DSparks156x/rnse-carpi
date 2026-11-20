# Desktop simulator (rnse-sim)

The car around a real carpid (and, on Linux, the real LIVI), on a PC: virtual RNS-E faceplate, wheel and stalk, the cluster DIS, and the car's state. Code: `tools/rnse-sim`.

## Paths in the car

### Keys: RNS-E / wheel / stalk to LIVI

```
RNS-E 0x461  ┐
wheel 0x5C3  ├─ ICAN ─▶ carpid: Decoder ─▶ Recogniser ─▶ route() ─┬─ Livi(key) ─┬─ focus "projection": Command::Input over the API ─▶ livi-carpi input::aa ─▶ AA keycodes
stalk 0x2C1  ┘          (carpi-input decode.rs, gesture.rs, route.rs) │            └─ otherwise: uinput "carpi-keys" ─▶ LIVI's key bindings
                                                                    ├─ Car(action) ─▶ Command::Input (OpenNav, OpenPage, CallAccept...) always over the API
                                                                    ├─ Dis(key) ─▶ DIS pages (focus on the DIS; stalk always)
                                                                    └─ Action ─▶ capture, shutdown, reboot
```

- 0x461 `37 30 SS K1 K2 00`: `SS` 01 press, repeats every 110 ms; 04 release ×5. Knob: one 01 frame per detent (`00 20` CW, `00 40` CCW).
- 0x5C3 `xx CC`: command while held (~500 ms cycle), `00` released. 0x2C1 byte 2: `0x20` up, `0x10` down.
- Gestures: faceplate long 440 ms (`[input] long_ms`), extended 10 s; wheel long 1 s. Bindings: `RouteConfig::default()` or `[input] bindings`.
- Keys route only while the RNS-E is on TV (0x661 byte 3 = `37`) and, except system actions, while LIVI is connected (it sent a `Bridge` update).
- Focus comes from LIVI (`BridgeUpdate::Focus` → `ui.focus`): `projection`, `livi`, `page:<name>`.
- Knob → AA: `Rotate { steps: ±1 }` per detent → `KnobRight/Left` → `Rotary(±1)`. AA never gets a delta above 1; speed is the event rate.
- Select → `DPAD_CENTER` down/up; Back → `BACK` down/up; D-pad left/right → down/up pairs; Home, media, call: clicks.

### Out to the car

| Where | Frames | From |
|---|---|---|
| Cluster centre (DIS) | DDP over TP2 on 0x6C0/0x6C1; 128x96 1-bit (white cluster) | `carpi-dis` via `carpi-core::display`: readings, nav (maneuver, distance, road, lanes, ETA from the bridge), media, call |
| Cluster top lines | 0x363 / 0x365, 8 AUDSCII cells each, only on TV | phone > nav > media, skipping what the centre shows |
| RNS-E | 0x7B0 `BA` label (LIVI/CarPlay/AA), `BB` brightness | `carpi-rnse`, re-sent when the RNS-E rejoins the NM ring |
| RNS-E screen | video (SDLink A/V, 800x480) | LIVI's UI; nothing over CAN |

Gates: DIS draws only with the NM ring awake and Kl15 on; every frame goes through the TX gate (`[tx] enabled`).

### Car values LIVI reads

`livi-carpi/src/values.rs` subscribes to: `vehicle.speed_kmh`, `vehicle.odometer_km`, `engine.rpm`, `engine.coolant_c`, `power.battery_v`, `fuel.*`, `brakes.*`, `trans.selector`, `lights.*`, `body.*`, `ambient.outside_temp_c`, `steering.angle_deg`, `light.night`, `light.dim_display_pct`. They become LIVI telemetry (AA sensors, night/dark mode). Wheel pulses (0x5B5) are decoded by carpid (`wheels.*`) but LIVI doesn't subscribe to them.

## The simulator

```
rnse-sim ─ hub (TCP virtual CAN, CARPI_SIM_CAN) ─ carpid (unchanged logic) ─ API ─ LIVI (real, Linux) / stand-in (Windows)
   └ SimCluster + Fb model ◀── DDP, top lines, 0x7B0
```

carpid's only sim hooks: `CARPI_SIM_CAN=host:port` makes `Port::open` / `TxPort::open` use the hub (`carpi-can/src/sim.rs`); `[api] socket = "tcp:host:port"` serves the API on TCP (Windows has no unix sockets for it).

### Windows

```
.\tools\rnse-sim\run.ps1              # window
.\tools\rnse-sim\run.ps1 --selftest   # headless: every path, PASS/FAIL, DIS PNGs in tools/rnse-sim/work
```

LIVI doesn't build for Windows; a stand-in takes its place on carpid's API (focus, projection, nav, media, call pickers) and shows what AA would get.

### Real LIVI (NixOS WSL + WSLg)

```
.\tools\rnse-sim\run-livi.ps1
```

Builds LIVI, carpid and rnse-sim inside the `NixOS` WSL distro (`wsl/shell.nix`, no root; first build ~20 min) and runs all three; LIVI's window is the RNS-E screen at 800x480. LIVI connects to carpid's unix socket as in the car. `--selftest` checks the LIVI paths headless (7 checks).

- LIVI's config is `~/rnse-sim-work/livi-config/LIVI/config.json`: copied from `~/.config/LIVI` the first time, with `knobLeft`/`knobRight` filled in as PageUp/PageDown when empty, as the car's first boot does (`os/options.py`). LIVI's own default binds no knob keys, so without it the knob does nothing in LIVI's menus.
- LIVI runs with `LIVI_NO_COMPOSITOR=1`: its nested compositor has no display under WSLg.
- uinput: WSLg's compositor doesn't read uinput, so the keys carpid sends to LIVI's menus (`key X -> LIVI` in its log) are pressed into LIVI's window from Windows (`wsl/livi-keys.ps1` via WSL interop: focus, key with scan code, focus back), with LIVI's own bindings (PageUp/PageDown for the knob). Projection keys go over the bridge as in the car.
- Android Auto needs a phone. USB: forward it with usbipd-win (`usbipd bind`, `usbipd attach --wsl`); untested here. Wireless AA needs LIVI's Wi-Fi AP and Bluetooth helper, which WSL doesn't have.

### Controls

| | |
|---|---|
| Faceplate | buttons are held while the mouse is down; keyboard: arrows = Prev/Next/Up/Down, Enter/Space = knob push, Backspace/Esc = Return |
| Knob | mouse wheel over it (one notch = one detent), `[` `]` (OS key repeat = turn speed), or Spin N detents at R/s |
| Wheel / stalk | buttons; keys `w` `s` `q` `e`, PgUp/PgDn |
| Timed press | any key for an exact hold (short 150 ms, long 700 ms, extended 10.5 s) |
| Car | bus awake, ignition, on TV, night, speed, steering (wheel speeds follow), gear, lights, doors, units |
| carpid | config in `tools/rnse-sim/work/carpid.toml`; edit, then Restart carpid |

Not simulated: diagnostic ECUs (TP2/KWP; the readings page shows `--` for diag values), the cluster's own screens and line 2 (outside temperature).
