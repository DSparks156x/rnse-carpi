# Build and run LIVI, carpid and rnse-sim on Linux (NixOS-WSL with WSLg, or
# any Nix host). No root needed: `nix-shell tools/rnse-sim/wsl/shell.nix`.
{ pkgs ? import <nixpkgs> { } }:
let
  gst = with pkgs.gst_all_1; [ gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-libav ];
  runtime = with pkgs; [
    wayland libxkbcommon fontconfig freetype libGL mesa libdrm libinput seatd udev
    openssl libva pulseaudio pipewire (pkgs.libgbm or pkgs.mesa) xorg.libX11 xorg.libXcursor xorg.libXi xorg.libXrandr
  ];
in
pkgs.mkShell {
  nativeBuildInputs = with pkgs; [ rustup pkg-config cmake gcc gnumake perl clang llvmPackages.libclang git ];
  buildInputs = gst ++ runtime;
  LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
  LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (gst ++ runtime);
  GST_PLUGIN_SYSTEM_PATH_1_0 = pkgs.lib.makeSearchPathOutput "lib" "lib/gstreamer-1.0" gst;
  shellHook = ''
    export RUSTUP_HOME=$HOME/.rustup CARGO_HOME=$HOME/.cargo
    export PATH=$CARGO_HOME/bin:$PATH
  '';
}
