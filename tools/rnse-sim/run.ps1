# One command: build carpid and the simulator, then start the simulator (which
# starts carpid on its virtual CAN bus). Extra arguments go to rnse-sim, e.g.
#   .\tools\rnse-sim\run.ps1 --selftest
#   .\tools\rnse-sim\run.ps1 --no-livi      # a real LIVI connects to carpid instead
$ErrorActionPreference = 'Stop'
$sim = $PSScriptRoot
$root = Resolve-Path (Join-Path $sim '..\..')
cargo build --manifest-path (Join-Path $root 'Cargo.toml') -p carpid
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cargo build --manifest-path (Join-Path $sim 'Cargo.toml')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& (Join-Path $sim 'target\debug\rnse-sim.exe') @args
exit $LASTEXITCODE
