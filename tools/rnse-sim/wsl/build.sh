#!/bin/sh
# Inside NixOS-WSL (or any Nix host): build LIVI, carpid and rnse-sim for Linux.
# Targets go to ~/rnse-sim-target (the Linux side's own disk; fast).
set -e
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
exec nix-shell "$here/shell.nix" --run "
  set -e
  rustup show active-toolchain >/dev/null 2>&1 || rustup default stable
  echo '== LIVI'; cd '$repo/livi/native' && CARGO_TARGET_DIR=\$HOME/rnse-sim-target/livi cargo build --release
  echo '== carpid'; cd '$repo' && CARGO_TARGET_DIR=\$HOME/rnse-sim-target/carpi cargo build -p carpid
  echo '== rnse-sim'; cd '$repo/tools/rnse-sim' && CARGO_TARGET_DIR=\$HOME/rnse-sim-target/sim cargo build
  echo '== done'
"
