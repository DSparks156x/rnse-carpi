//! The instrument cluster: the white PQ35 cluster's side of DDP (carpi-sim's
//! `SimCluster`, written independently of carpid's driver), what it would
//! show in the 128x96 1-bit centre (carpi-dis's `Fb` model, fed the drawing
//! messages the cluster accepted), and the two 8-cell top lines from
//! 0x363/0x365.

use carpi_can::Frame;
use carpi_dis::fb::{Fb, PX_H, PX_W};
use carpi_dis::font::{Font, Metrics};
use carpi_dis::proto::Variant;
use carpi_sim::bus::Node;
use carpi_sim::ddp::{ClusterSpec, SimCluster};
use std::time::Instant;

pub const TOP_CELLS: usize = 8;
/// One top-line cell: the cluster's 12x14 fixed glyph.
pub const CELL_W: usize = carpi_dis::glyphs::W;
pub const CELL_H: usize = carpi_dis::glyphs::H;

pub struct Cluster {
    cl: SimCluster,
    fb: Fb,
    /// Top lines as AUDSCII cells; `None` until the first frame.
    top: [Option<[u8; 8]>; 2],
    /// Drawing messages applied, and the cluster's notes.
    pub messages: u64,
    pub violations: Vec<String>,
}

impl Cluster {
    pub fn new() -> Self {
        Cluster {
            cl: SimCluster::new(ClusterSpec::default()),
            fb: Fb::new(Metrics::for_variant(Variant::White)),
            top: [None, None],
            messages: 0,
            violations: Vec::new(),
        }
    }

    pub fn on_frame(&mut self, now: Instant, f: &Frame) {
        match f.id {
            carpi_sim::ddp::HU_ID => self.cl.on_frame(now, f),
            carpi_dis::top::TOP1_ID | carpi_dis::top::TOP2_ID => {
                let line = usize::from(f.id == carpi_dis::top::TOP2_ID);
                let mut cells = [0x20u8; 8];
                for (c, b) in cells.iter_mut().zip(f.data()) {
                    *c = *b;
                }
                self.top[line] = Some(cells);
            }
            _ => {}
        }
    }

    /// Runs the cluster's timers; its frames go to `out`.
    pub fn tick(&mut self, now: Instant, out: &mut Vec<Frame>) {
        self.cl.tick(now);
        while let Some(f) = self.cl.poll_transmit(now) {
            out.push(f);
        }
        // Apply what it accepted, then drop the history so it can't grow.
        for m in self.cl.drawn.drain(..) {
            self.fb.apply(&m);
            self.messages += 1;
        }
        self.cl.received.clear();
        if !self.cl.violations.is_empty() {
            self.violations.append(&mut self.cl.violations);
            let n = self.violations.len();
            if n > 20 {
                self.violations.drain(..n - 20);
            }
        }
    }

    pub fn owned(&self) -> bool {
        self.cl.owner
    }

    pub fn open(&self) -> bool {
        self.cl.is_open()
    }

    pub fn frozen(&self) -> bool {
        self.cl.frozen
    }

    /// The top lines' raw cells, for logs.
    pub fn top_hex(&self) -> String {
        self.top
            .iter()
            .map(|l| match l {
                Some(c) => c
                    .iter()
                    .map(|b| format!("{b:02X}"))
                    .collect::<Vec<_>>()
                    .join(" "),
                None => "-".into(),
            })
            .collect::<Vec<_>>()
            .join(" / ")
    }

    pub fn has_top(&self) -> bool {
        self.top.iter().any(Option::is_some)
    }

    /// The centre, 128x96, lit = true.
    pub fn centre(&self) -> &[bool] {
        self.fb.pixels()
    }

    /// Both top lines as pixels: 96x28, lit = true.
    pub fn top_pixels(&self) -> Vec<bool> {
        let w = TOP_CELLS * CELL_W;
        let mut px = vec![false; w * CELL_H * 2];
        for (line, cells) in self.top.iter().enumerate() {
            let Some(cells) = cells else { continue };
            for (i, &code) in cells.iter().enumerate() {
                for gy in 0..CELL_H {
                    let bits = carpi_dis::glyphs::row(Font::Fixed, code, gy);
                    for gx in 0..CELL_W {
                        if bits & (0x8000 >> gx) != 0 {
                            px[(line * CELL_H + gy) * w + i * CELL_W + gx] = true;
                        }
                    }
                }
            }
        }
        px
    }
}

pub const CENTRE_W: usize = PX_W;
pub const CENTRE_H: usize = PX_H;
