# NaxGCC

This repo houses the firmware for the NaxGCC, a GameCube-style controller built on the [PhobGCC](https://github.com/PhobGCC/PhobGCC-SW). The firmware can also be used as an optional firmware for the PhobGCC, though the PhobGCC will then have to be connected to the console directly via USB.

Like the PhobGCC, the NaxGCC uses hall effect sensors instead of potentiometers for stick input. Additionally, it connects directly to the console via USB, by pretending to be a GCC adapter with 1 controller (itself) connected. This eliminates one additional layer of polling, and thus reduces perceived latency and improves input consistency. The NaxGCC firmware makes use of the [embassy-rs](https://github.com/embassy-rs/embassy) framework for asynchronous operations. Mainly, this means that the firmware is capable of polling the sticks and buttons at different frequencies, further improving input consistency for button inputs.

## What's the deal with polling?

### General Info

The vast majority of human interface devices (HIDs) transmit their states using a technique known as "polling". Essentially, these devices (clients) advertise themselves as supporting a certain polling frequency to whatever device they're attached to (hosts). The host then polls the client at the desired frequency, or at a lower one (= not as often, = more slowly) if the host doesn't support the client's desired frequency.

The Nintendo Switch supports polling USB devices at up to 125Hz, or once every 8ms. A game running at 60fps takes ~16.66ms to draw a frame, meaning with a polling rate of 8ms, a USB device would be able to update its state _up to_ 3 times per frame. Notice how I said "up to", because this is where things start to get wonky.

The reason it's "up to 3 times" and not "3 times, period" is because the USB polling and the game's frame draws are not in sync. Otherwise, a polling rate of ~16.66ms would be perfectly sufficient, provided the host polls the adapter right before the frame is supposed to be drawn. Due to technical reasons, this is not possible however, so we are stuck with two asynchronous intervals: Polling & frame draws.

### Input Integrity

Now, what does this mean for your input integrity? Essentially, the time window in which your inputs are _guaranteed_ to come out on the frame you'd expect them to is equal to $T_f-T_p$ where $T_f$ is the time it takes to draw a frame (~16.66ms) and $T_p$ is the polling interval (8ms). So, on the Nintendo Switch, where USB polling is locked 8ms and frame time is 16.66ms, our window of time during which inputs are _guaranteed_ to come out on the _expected_ frame is $16.\overline{6} - 8 = 8.\overline{6}ms$

This then means that during the first ~8.66ms of a "frame capture window", your input is guaranteed to come out on the frame you'd expect (the one whose frame capture window is currently open). For any input, the probability $p(n)$ that your input will arrive at the expected frame, for any $n$ that is the time elapsed (in ms) since the start of the frame capture window, is $p(n) = \frac{T_f - n}{T_p}$. As you can see, for $n \leq T_f - T_p, p(n) \geq 1 \rightarrow p(n) = 1$, and for $n \gt T_f - T_p, p(n) \lt 1$.

In plain English, with a game running at 60fps and USB polling at 8ms, you have an ~8.66ms (slightly more than half a frame) window where your inputs are guaranteed to be consistent, everything outside that window is RNG to varying degrees, whether your input will be delayed by a frame or not.

Now this sounds kind of bad at first, but keep in mind that for the first few milliseconds before and after this "golden" window, the likelihood is still very high ($\geq 75\%$) that your input will arrive at the frame you intended it for, so in total you will have a $\approx 12.66$ ms window where your inputs can reasonably be assumed to arrive at the frame you intended (sampling a random point from this 12.66ms window has a chance of $\gt 92\%$ of landing on the correct frame).

In reality, there will still be a little bit of RNG, and you won't be able to eliminate it fully, at least not with 8ms polling, which unfortunately is a limitation on the console side, but more than half a frame of guaranteed input integrity, and ~12.66ms of "reasonable" input integrity is something that I, as a competitor, can live with.

However, it's not that simple...

### Joybus

See, the math above assumes that the GCC adapter is the device providing the inputs to the console. However, it is only a middleware, and the true source of your inputs is your controller. The GameCube controller interacts with the GCC adapter pretty much the same as with the OG GameCube, using the [joybus](https://www.int03.co.uk/crema/hardware/gamecube/gc-control.html) protocol, which is the same protocol that N64 controller use. And its age shows, it doesn't have differential signalling, checksumming, or any of the other goodies other, more modern protocols (like USB) have.

But worst of all, it comes with yet another polling rate, one of 6ms. Now, you might be thinking to yourself _"but Naxdy, 6ms is less than 8ms, so surely this is a good thing?"_ well yes, but no. It _would_ be a good thing, if the GCC could be connected to the Nintendo Switch directly, but it cannot, it _has_ to go through the adapter, which has its own polling rate of 8ms.

So, you end up with a system with three independent polling rates: The Switch polls the adapter at 8ms intervals, and the adapter polls the controller at 6ms intervals, and then the controller has whatever scan rate it has (Phobs have a 1ms scan interval FYI). Now, remember how the time frame in which your input is _guaranteed_ to come out on the expected frame was $T_f - T_p$, but what about a system with multiple polling rates? Well, in this case it's $T_f - \sum_{i=1}^nT_i$ where $n$ is the number of individual polling rates and $T_1, T_2, ..., T_n$ are the individual polling rates (in ms).

Again in plain English, if you have multiple polling rates, the frame window in which your inputs are _guaranteed_ to come out on the frame you'd expect them to, is the total time of the frame window minus the sum of all polling rates. So, let's do some addition and subtraction for our use case here: $16.\overline{6} - 8 - 6 - 1 = 1.\overline{6}$. Now, you don't have to be a Harvard graduate to recognize that ~1.66ms is a teeny tiny window of time compared to the (theoretically perfect) ~8.66ms from before.

This is how it is when you're playing with a PhobGCC (the best GCC currently available) on a _first party_ GCC adapter from Nintendo. Note that third party adapters may very well be much worse than this, because they could poll the GCC at an even lower frequency (= more slowly).

**BONUS QUESTION:** What if the sum of all polling rates is larger than the frame time window, i.e. $\sum_{i=1}^nT_i > T_f$ ? That's right, in this case your inputs are _always_ RNG! (good thing that's not the case here though)

### The Solution

Since the NaxGCC connects directly to the console and eliminates the joybus protocol entirely, there is no second polling rate. The scan rate of the NaxGCC's sticks is 1ms, and the buttons are scanned as quickly as the MCU allows (I've measured ~200us -ish). While not quite reaching the ~8.66ms window length, the sticks have a ~7.66ms window of guaranteed input integrity, and the buttons are getting fairly close to ~8.3ms (half a frame).
