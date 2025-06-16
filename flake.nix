{
  inputs = {
    nixpkgs.url = "github:02alexander/nixpkgs";
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
        in
        {
          devShells.default = pkgs.mkShell {
            buildInputs = with pkgs; [
              (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml)
              (python313.withPackages (ps: with ps; [numpy scipy matplotlib control jupyter]))
              elf2uf2-rs
              probe-rs
              picotool
              udev
            ];
          };
        }
      );
}