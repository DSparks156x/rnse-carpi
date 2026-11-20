# One command, real LIVI: builds (first time ~20 min) and runs rnse-sim, carpid
# and LIVI inside the NixOS WSL distro; their windows appear through WSLg.
# Extra arguments go to rnse-sim, e.g. --selftest.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$wslRepo = (wsl.exe -d NixOS -- wslpath -a ($repo.Replace('\', '/'))).Trim()
$rest = $args -join ' '
wsl.exe -d NixOS -- sh -lc "cd '$wslRepo' && sh tools/rnse-sim/wsl/build.sh && sh tools/rnse-sim/wsl/run.sh $rest"
exit $LASTEXITCODE
