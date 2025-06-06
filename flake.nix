{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };
  outputs = { self, nixpkgs, flake-utils, rust-overlay}:
    flake-utils.lib.eachDefaultSystem
      (system: 
        let 
          overlays = [(import rust-overlay)];
          pkgs = import nixpkgs {
            inherit overlays system;
          };
          libs = with pkgs; [
              # glui
              # libGL
              gcc-unwrapped.lib
              libz
              glib
              # vulkan-loader
              # libxkbcommon
              # xorg.libX11
              # xorg.libXcursor
              # xorg.libXrender
              # xorg.libXi
              # xorg.libXrandr
              # xorg.libxcb
              # fontconfig
              # freetype
              # dbus
          ];
        in
        {
          devShells.default = pkgs.mkShell {
            # buildInputs = with pkgs; [(rust-bin.stable.latest.default)];
            buildInputs = with pkgs; [
              (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml)
              python312Full
              gnumake
              cmake
              ninja
              gfortran
              elf2uf2-rs
              probe-rs
              picotool
              udev
              pkg-config
            ] ++ libs;
            LD_LIBRARY_PATH= pkgs.lib.makeLibraryPath libs;
            shellHook = ''
              source analysis/venv/bin/activate
            '';
          };
        }
      );
}