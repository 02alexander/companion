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
              gcc-unwrapped.lib
              libz
              glib
              blas
          ];
        in
        {
          # devShells.default = (pkgs.buildFHSEnv {
          #   name="hello";
          #   packages = ps: with ps; [
          #     (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml)
          #     python312Full
          #     gnumake
          #     cmake
          #     ninja
          #     gfortran
          #     elf2uf2-rs
          #     probe-rs
          #     picotool
          #     udev
          #     pkg-configgcc-unwrapped.lib
          #     libz
          #     glib
          #   ];
          # }).e/nv;
          devShells.default = pkgs.mkShell {
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