{
  description = "Firmware for the NaxGCC";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-23.11";

    rust-overlay.url = "github:oxalica/rust-overlay";

    flake-utils.url = "github:numtide/flake-utils";

    embassy-rs-patched = {
      url = "git+ssh://gitea@git.naxdy.org/NaxdyOrg/embassy?ref=naxgcc-fw";
      flake = false;
    };

    naersk = {
      url = "github:nmattia/naersk";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    { self
    , nixpkgs
    , rust-overlay
    , flake-utils
    , naersk
    , embassy-rs-patched
    }: (flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
        overlays = [
          rust-overlay.overlays.default
        ];
        config.allowUnsupportedSystem = true;
      };

      rustToolchain = pkgs.rust-bin.stable.latest.default.override {
        extensions = [
          "rust-src"
        ];
        targets = [
          "thumbv6m-none-eabi"
        ];
      };

      naersk_lib = naersk.lib.${system}.override {
        cargo = rustToolchain;
        rustc = rustToolchain;
      };

      CARGO_BUILD_TARGET = "thumbv6m-none-eabi";

      prepCmd = ''
        mkdir -p lib
        rm lib/embassy-rs || true
        ln -s "${embassy-rs-patched}" lib/embassy-rs
      '';
    in
    {
      packages.default = self.packages.${system}.naxgcc-fw-uf2;

      packages.naxgcc-fw-uf2 = pkgs.runCommandLocal "${self.packages.${system}.naxgcc-fw.pname}-uf2-${self.packages.${system}.naxgcc-fw.version}" { } ''
        mkdir -p $out/bin
        ${pkgs.elf2uf2-rs}/bin/elf2uf2-rs ${self.packages.${system}.naxgcc-fw}/bin/${self.packages.${system}.naxgcc-fw.pname} $out/bin/${self.packages.${system}.naxgcc-fw.pname}.uf2
      '';

      packages.naxgcc-fw = naersk_lib.buildPackage {
        pname = (builtins.fromTOML (builtins.readFile ./Cargo.toml)).package.name;
        version = (builtins.fromTOML (builtins.readFile ./Cargo.toml)).package.version;

        prePatch = prepCmd;

        src = self;

        cargoBuildOptions = _orig: _orig ++ [
          "--target=${CARGO_BUILD_TARGET}"
        ];

        # inherit RUSTFLAGS;
      };

      devShells.default = pkgs.mkShell {
        nativeBuildInputs = builtins.attrValues {
          inherit rustToolchain;
          inherit (pkgs) gcc-arm-embedded elf2uf2-rs picotool probe-rs cargo-expand;
        };

        CARGO_TARGET_THUMBV6M_NONE_EABI_RUNNER = "probe-rs run --chip RP2040 --protocol swd";
        DEFMT_LOG = "debug";

        inherit CARGO_BUILD_TARGET;

        shellHook = prepCmd;
      };
    }));
}
