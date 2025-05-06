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
        in
        {
          # packages.default = derivation {
          #   name="test";
          #   inherit system;
          #   builder = "${pkgs.bash}/bin/bash";
          #   args = ["-c" "echo ${builtins.concatStringsSep " " (builtins.attrNames pkgs.rust-bin)} > $out"];
          # };

          devShells.default = pkgs.mkShell {
            # buildInputs = with pkgs; [(rust-bin.stable.latest.default)];
            buildInputs = with pkgs; [
              (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml)
              elf2uf2-rs
              probe-rs
              picotool
            ];
          };
        }
      );
}