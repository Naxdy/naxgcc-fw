# Contributing

If you'd like to contribute to the development of the NaxGCC firmware, you're in the right place! This document will guide you through the process of setting up your development environment, building the firmware, testing it on your controller, and submitting your changes for review.

## Getting Started

The firmware is built using [nix](https://nixos.org/), a package manager that provides a consistent development environment across different systems. The easiest way for you to get going is to install nix on your system, [enable flakes](https://nixos.wiki/wiki/Flakes), and then run the following command:

```sh
nix develop .
```

This will begin downloading all the necessary development tools, such as Rust and the ARM toolchain, and will drop you into a shell where you can immediately run `cargo build` to build the firmware, without any additional setup. From this shell, you can also run your favorite text editor, and it will also have access to all the necessary tools.

If you want to take this a step further, you can install [direnv](https://direnv.net/), which will automatically drop you into a development shell whenever you enter the project directory. It also has a corresponding [VSCode extension](https://marketplace.visualstudio.com/items?itemName=mkhl.direnv) that will automatically set up your environment when you open the project in VSCode/VSCodium.

### Building the firmware

To build the firmware, simply run:

```sh
cargo build
```

for a development build, and

```sh
nix build .#
```

for a release build. Release builds will be placed in `./result/bin/` and can be flashed to your controller by simply dragging & dropping the `.uf2` file onto the controller while it's in bootloader mode (press `A+X+Y` while plugging it in).

### Debugging

The NaxGCC board exposes all the necessary pins to hook up a Pico debug probe. Running

```sh
cargo run
```

will automatically look for connected debug probes and use the first one it finds to flash the firmware to the controller.

## Submitting your changes

When you're ready to submit your changes, simply push your branch to the repository and open a pull request. The CI will automatically run tests on your changes, and a maintainer will review your code. If everything looks good, your changes will be merged into the main branch.

### Things not to submit

We strive to keep the NaxGCC firmware as clean and minimal as possible. As such, we have a few guidelines for contributions:

- **No Melee-specific features**: The primary focus of the NaxGCC firmware is Smash Ultimate and other Nintendo Switch games. As such, features that are solely beneficial for emulated Melee gameplay (e.g. analog trigger functionality) will probably be rejected. Keep in mind that you cannot use a NaxGCC on an actual GameCube/Wii anyway!
- **No time-based macros or other automation**: The NaxGCC firmware is designed to be tournament-legal, and as such, we do not allow any form of automation or macros that could give players an unfair advantage. Input filters and other simple "if-then" logic is acceptable (see [input_filter.rs](https://git.naxdy.org/NaxdyOrg/NaxGCC-FW/src/branch/main/src/input_filter.rs) for examples), but anything more complex will be rejected. The rule of thumb is: If it can be reasonably implemented as a hardware mod, it's probably fine.
- **No feature bloat**: We aim to keep the firmware as minimal as possible, to reduce the risk of bugs and to keep the codebase maintainable. As such, we will reject any features that are not deemed essential for the core functionality of the controller. This is especially important since the RP2040 executes the majority of code from flash, thus binary size matters. Performance improvements are always welcome, though.
