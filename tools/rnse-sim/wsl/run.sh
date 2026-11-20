#!/bin/sh
# Inside NixOS-WSL: rnse-sim + carpid + the real LIVI, after build.sh.
# Windows one-liner: tools\rnse-sim\run-livi.ps1
set -e
here=$(cd "$(dirname "$0")" && pwd)
t=$HOME/rnse-sim-target
exec nix-shell "$here/shell.nix" --run "
  '$t/sim/debug/rnse-sim' \
    --carpid '$t/carpi/debug/carpid' \
    --livi '$t/livi/release/livi-core' \
    --work \$HOME/rnse-sim-work $*
"
