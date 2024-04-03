# NaxGCC

This repo houses the firmware for the NaxGCC, a GameCube-style controller built on the [PhobGCC](https://github.com/PhobGCC/PhobGCC-SW). The firmware can also be used as an optional firmware for the PhobGCC, though the PhobGCC will then have to be connected to the console directly via USB.

Like the PhobGCC, the NaxGCC uses hall effect sensors instead of potentiometers for stick input. Additionally, it connects directly to the console via USB, by pretending to be a GCC adapter with 1 controller (itself) connected. This eliminates one additional layer of polling, and thus reduces perceived latency and improves input consistency. The NaxGCC firmware makes use of the [embassy-rs](https://github.com/embassy-rs/embassy) framework for asynchronous operations. Mainly, this means that the firmware is capable of polling the sticks and buttons at different frequencies, further improving input consistency and latency for button inputs.

### Key Aspects

Click on any of these to expand.

<details><summary>NaxGCC has all the important PhobGCC features.</summary>

The hardware of NaxGCC is directly forked from PhobGCC's, meaning it benefits from the same improvements over a "regular" GCC, most importantly the fact that it uses hall-effect sensors instead of potentiometers for reading your stick positions.

Furthermore, large parts of its firmware have also been taken from PhobGCC's firmware, such as the snapback filter, cardinal snapping, and notch remapping to name a few. If you're used to calibrating a PhobGCC, you will have no trouble here.

</details>

<details><summary>Firmware is written in Rust, using the <a href="#">embassy-rs</a> framework for asynchronous operations.</summary>

The firmware being written in Rust allows for writing much cleaner code than one would normally be used to when writing firmware in C, because Rust allows for many zero and low cost abstractions in order to enhance code readability and maintainability. Adding embassy-rs for asynchronous operations on top of that provides 2 main benefits:

1. It further improves code readability and maintainability by allowing to separate functionality on a semantic level.
2. It allows multiple tasks to be executed on the same thread, sharing their workload. Effectively, due to this, the NaxGCC can update its buttons at a ~50us (that's *micro*seconds) interval, and its sticks at a 1ms interval.

</details>

<details><summary>Provides both the lowest latency of any Switch controller, as well as the best input integrity.</summary>

Because the NaxGCC connects directly to the console via USB, it already outperforms any controller that has to go through an adapter in terms of input latency (including PhobGCC + Lossless Adapter).

Further, the NaxGCC has a special "input consistency" mode (enabled by default), which ensures a $\gt 98\%$ input accuracy, compared to $\lt 75\%$ for any other controller (worse if there is an adapter in the mix, with the exception of the Lossless Adapter).

For details on how it works, have a look at our wiki.

</details>

<details><summary>Compatible with Phob hardware.</summary>

The NaxGCC firmware is compatible with regular Phob 2.0 boards (those using an RP2040 microcontroller), since it's originally forked from the PhobGCC project. This means that if you are willing and able to slightly modify your controller shell to allow a micro USB cable to connect to your Phob board during play, you can turn your existing PhobGCC into a NaxGCC at no extra cost!

</details>

## Contributing

The NaxGCC firmware is built using [nix](), which also provides a ready-to-go development environment, complete with all the tooling and libraries you need to get going. Simply install nix, [enable flakes]() and run

```bash
nix develop .
```

and you're ready to work on the project. Submit your pull requests [here](https://git.naxdy.org/NaxdyOrg/NaxGCC-FW/pulls).
