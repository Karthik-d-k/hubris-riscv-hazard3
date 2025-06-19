## Install

### Windows

1. A `rustup`-based toolchain install. `rustup` will take care of automatically
  installing our pinned toolchain version, and the cross-compilation targets,
  when you first try to build.

2. `openocd` (ideally 0.11) or (if using the LPC55) `pyocd` (0.27 or later)
   - Install from [GitHub releases](https://github.com/openocd-org/openocd/releases/tag/v0.11.0)

3. To use the ST-Link programmer, you'll probably need to install
[this driver](https://www.st.com/en/development-tools/stsw-link009.html).

4. To use GDB
    - Install `arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi.zip` and add to `PATH` Environment variable
    - Windows (mingw-w64-x86_64) hosted cross toolchains
    - AArch32 bare-metal target (arm-none-eabi)
    - [official ARM binaries](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

5. Hubris debugger, [Humility](https://github.com/oxidecomputer/humility)
    - Install from a directory where `rust-toolchain.toml` is not present.
    ```sh
    cargo install cargo-readme
    cargo install --git https://github.com/oxidecomputer/humility.git --locked humility-bin
    ```

### Linux (Ubuntu)

1. A `rustup`-based toolchain install. `rustup` will take care of automatically
  installing our pinned toolchain version, and the cross-compilation targets,
  when you first try to build.

2. `openocd` (ideally 0.11) or (if using the LPC55) `pyocd` (0.27 or later)
    - Better to install it from source as system package manager isntalls 0.10 version
    ```sh
    sudo apt update
    sudo apt install make libtool pkg-config autoconf automake texinfo libjim-dev
    git clone http://repo.or.cz/r/openocd.git
    cd openocd/
    ./bootstrap
    ./configure --enable-stlink=yes
    make
    sudo make install
    ```

3. [libusb](https://libusb.info/)
    ```sh
    sudo apt install libusb-1.0-0-dev libusb-1.0-0
    ```

4. [libfdti1](https://www.intra2net.com/en/developer/libftdi/)
    ```sh
    sudo apt install libftdi1 libftdi1-dev
    ```

5. To use GDB
    ```sh
    sudo apt install gdb-multiarch
    ```

6. Hubris debugger, [Humility](https://github.com/oxidecomputer/humility)
    - Install from a directory where `rust-toolchain.toml` is not present.
    ```sh
    cargo install cargo-readme
    cargo install --git https://github.com/oxidecomputer/humility.git --locked humility-bin
    ```

7.  In WSL, to use USB to flash and everything, we need to do additional set-up as explained in [Connect USB devices on WSL](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)


## Build

**We do not use `cargo build` or `cargo run` directly because they are too
inflexible for our purposes.** We have a complex multi-architecture build, which
is a bit beyond them.

Instead, the repo includes a Cargo extension called `xtask` that namespaces our
custom build commands.

#### Full Image

`cargo xtask dist TOMLFILE` builds a **distribution image** for the
application described by the TOML file.

```sh
cargo xtask dist app/demo-stm32f4-discovery/app.toml` - stm32f4-discovery
```

#### Tasks

To build `task-ping` as it would be built in one of the images, but
without building the rest of the demo, run:

```sh
cargo xtask build app/demo-stm32f4-discovery/app.toml ping
```


## Flash

An image within a Hubris archive can be flashed directly onto a target board
by running `cargo xtask flash` and specifying the appropriate
TOML file.  This will run `cargo xtask dist` and then pass the resulting
build archive to `humility flash`. `humility` will invoke either OpenOCD or
pyOCD to flash the image; the exact invocation depends on the board
and is encoded in the build archive.

- STM32F4 Discovery board:
```sh
cargo xtask flash app/demo-stm32f4-discovery/app.toml
```


## Running Humility

Humility is run _in situ_ by specifying an archive on a directly connected
board, or postmortem by specifying a dump.  As a convenience for development,
Humility can also be run _in situ_ by specifying the appropriate TOML, e.g.
on a machine with an STM32F4 Discovery board directly attached:

```sh
cargo xtask humility app/demo-stm32f4-discovery/app.toml -- tasks
    Finished dev [optimized + debuginfo] target(s) in 0.17s
     Running `target/debug/xtask humility demo/app.toml -- tasks`
humility: attached via ST-Link
ID ADDR     TASK               GEN STATE    
 0 20000108 jefe                 0 Healthy(InRecv(None))     
 1 20000178 rcc_driver           0 Healthy(InRecv(None))     
 2 200001e8 usart_driver         0 Healthy(InRecv(None))     
 3 20000258 user_leds            0 Healthy(Runnable)          <-
 4 200002c8 ping                48 Healthy(Runnable)         
 5 20000338 pong                 0 Healthy(InRecv(None))     
 6 200003a8 idle                 0 Healthy(Runnable)         
```


## Debugging with GDB

`humility` includes a `gdb` subcommand which attaches to a running system
using `arm-none-eabi-gdb`, optionally running its own `openocd` instance based
on configuration data in the build archive.

For convenience, there's also a `cargo xtask gdb` façade which calls `humility`
with the appropriate build archive:

```sh
cargo xtask gdb app/demo-stm32f4-discovery/app.toml -- --run-openocd --gdb-script .\chips\stm32f4\openocd.gdb
# ... lots of output elided ...
task_idle::main () at task/idle/src/main.rs:14
14          loop {
Breakpoint 1 at 0x800434c: file /crates.io/cortex-m-rt-0.6.15/src/lib.rs, line 560.
Note: automatically using hardware breakpoints for read-only addresses.
semihosting is enabled

semihosting is enabled

(gdb)
```

Note that `cargo xtask gdb` will (by default) also run `dist` and `flash`, to
ensure that the image on the chip is up to date.  The `-n`/`--noflash` option
skips these steps.


## Testing Hubris

The Hubris kernel is tested with a dedicated _test image_ that includes a test
runner, assistant and test suite.  The test image emits its results via ITM.
While these results can be interpreted manually, `humility test` automates
this.  `humility test` itself is most easily run via `cargo xtask test`, which
runs the equivalent of `cargo xtask dist`, `cargo xtask flash`
and `cargo xtask humility test`.  The exact invocation depends on the board:

- STM32F3 Discovery board: 
```sh
cargo xtask test test/tests-stm32fx/app-f3.toml
```

Note: `cargo xtask humility test` runs OpenOCD to connect to the device.
You must exit any other instances of OpenOCD that you have connected to the device
before running tests.

All tests will produce an output file. This contains information about the hubris archive as well as task state after each test run. Search for "fail" to see any failed tests.

```sh
cat hubris.testout.1
```


## Graphing task relationships and priorities

A graph can be generated that show the relationships of the various tasks
and their priorities. The resulting file is in [Graphviz](https://graphviz.org/)'s
`dot` format. `Dot` source [can be included](https://docs.asciidoctor.org/diagram-extension/latest/) in [Asciidoctor](https://asciidoctor.org) source
or rendered to a variety of formats.

To create and view an SVG graph for `stm32f4` on Windows, ensure that the `graphviz` package is installed. Then generate the graph:

```sh
cargo xtask graph -o stm32f4.dot app/demo-stm32f4-discovery/app.toml
dot -Tsvg stm32f4.dot > stm32f4.svg
```


## Adding a new task

To create your own task, the easiest method is:

- Copy `task/template` to a new name.
- Edit its `Cargo.toml` with your name and a new package name.
- Add it to the list of workspace members in the root `Cargo.toml`.
- Add it to a system image by editing an `app.toml` file.
- Run `cargo xtask build` to compile it.

A typical `app.toml` entry for a small task that uses no memory-mapped
peripherals would read

```toml
[tasks.name_for_task_in_this_image]
name = "my-task-target-name"
priority = 1
requires = {flash = 1024, ram = 1024}
start = true
```


## Running `clippy`

The `cargo xtask clippy` subcommand can be used to run `clippy` against one or
more tasks in the context of a particular image:

```sh
cargo xtask clippy app/demo-stm32f4-discovery/app.toml ping pong
```


## References

01. [hubris-fork](https://github.com/Karthik-d-k/hubris)
02. [Hubris Reference](https://hubris.oxide.computer/reference/)
03. [exhubris-repo](https://github.com/cbiffle/exhubris)
04. [exhubris-demo-repo](https://github.com/cbiffle/exhubris-demo/)
05. [jperkin-riscv-fork](https://github.com/oxidecomputer/hubris/compare/master...jperkin:hubris:riscv)
06. [On Hubris And Humility](https://cliffle.com/blog/on-hubris-and-humility/)
07. [From Hubris To Bits](https://cliffle.com/blog/from-hubris-to-bits/)
08. [exhubris: Revisiting Hubris appconfigs](https://cliffle.com/blog/exhubris/)
09. [exhubris: Crash recovery in 256 bytes](https://cliffle.com/blog/exhubris-super/)
10. [exhubris-demo-repo](https://github.com/cbiffle/exhubris-demo/)
11. [humility-repo](https://github.com/oxidecomputer/humility)
