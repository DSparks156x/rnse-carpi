//! Minimal PNG writer (grayscale, stored deflate) for self-test snapshots.

fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= u32::from(b);
        for _ in 0..8 {
            c = if c & 1 != 0 {
                0xEDB8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
        }
    }
    !c
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &x in data {
        a = (a + u32::from(x)) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let mut c = kind.to_vec();
    c.extend_from_slice(data);
    out.extend_from_slice(&c);
    out.extend_from_slice(&crc32(&c).to_be_bytes());
}

/// `px` lit/unlit, `w`x`h`, each pixel scaled `scale` times.
pub fn encode(px: &[bool], w: usize, h: usize, scale: usize) -> Vec<u8> {
    let (sw, sh) = (w * scale, h * scale);
    let mut raw = Vec::with_capacity((sw + 1) * sh);
    for y in 0..sh {
        raw.push(0);
        for x in 0..sw {
            raw.push(if px[(y / scale) * w + x / scale] {
                0xF0
            } else {
                0x00
            });
        }
    }
    finish(raw, sw, sh, 0)
}

/// RGBA, 8 bits per channel.
pub fn encode_rgba(rgba: &[u8], w: usize, h: usize) -> Vec<u8> {
    let mut raw = Vec::with_capacity((w * 4 + 1) * h);
    for row in rgba.chunks(w * 4).take(h) {
        raw.push(0);
        raw.extend_from_slice(row);
    }
    finish(raw, w, h, 6)
}

fn finish(raw: Vec<u8>, sw: usize, sh: usize, color: u8) -> Vec<u8> {
    let mut z = vec![0x78, 0x01];
    for (i, block) in raw.chunks(65_535).enumerate() {
        let last = (i + 1) * 65_535 >= raw.len();
        z.push(u8::from(last));
        let n = block.len() as u16;
        z.extend_from_slice(&n.to_le_bytes());
        z.extend_from_slice(&(!n).to_le_bytes());
        z.extend_from_slice(block);
    }
    z.extend_from_slice(&adler32(&raw).to_be_bytes());
    let mut out = vec![0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A];
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(sw as u32).to_be_bytes());
    ihdr.extend_from_slice(&(sh as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, color, 0, 0, 0]);
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}
