#!/bin/sh
# Linux/WSL/macOS: build carpid and the simulator, then start it.
set -e
sim=$(cd "$(dirname "$0")" && pwd)
cargo build --manifest-path "$sim/../../Cargo.toml" -p carpid
cargo build --manifest-path "$sim/Cargo.toml"
exec "$sim/target/debug/rnse-sim" "$@"
