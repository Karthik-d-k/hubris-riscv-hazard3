# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Hubris is a microcontroller operating environment designed for deeply-embedded systems with reliability requirements. It's a Rust-based microkernel with drivers running in unprivileged mode, used for server firmware on Oxide Computer hardware (Gimlet, Sidecar, PSC, etc.) and various development boards.

Key characteristics:
- Pure Rust embedded firmware (no_std)
- Priority-based preemptive scheduling with task isolation
- Tasks are defined at compile-time (no dynamic task creation)
- Inter-task communication via IPC using the Idol IDL system
- ARM Cortex-M targets (thumbv6m, thumbv7em, thumbv8m)

## Build Commands

**Important:** Do not use `cargo build` or `cargo run` directly. Use the `xtask` build system.

```bash
# Build complete system image
cargo xtask dist app/<app-name>/app.toml

# Build single task (faster iteration)
cargo xtask build app/<app-name>/app.toml <task-name>

# List all tasks in an image
cargo xtask build app/<app-name>/app.toml --list

# Run clippy on specific tasks
cargo xtask clippy app/<app-name>/app.toml <task-names...>

# Flash image to connected board
cargo xtask flash app/<app-name>/app.toml

# Debug with GDB
cargo xtask gdb app/<app-name>/app.toml -- --run-openocd

# Run tests on hardware
cargo xtask test test/<test-image>/app.toml

# Run humility debugger
cargo xtask humility app/<app-name>/app.toml -- <humility-args>

# Print sizes of tasks
cargo xtask sizes app/<app-name>/app.toml

# Generate task dependency graph (Graphviz dot format)
cargo xtask graph -o output.dot app/<app-name>/app.toml

# Configure rust-analyzer for a file
cargo xtask lsp <rust-file>
```

Standard Cargo commands for workspace-level checks:
```bash
cargo fmt --all --check
cargo test --workspace
```

## Architecture

### Directory Structure

- `app/` - Top-level binary crates for firmware images (e.g., `app/gimlet/`, `app/demo-stm32f4-discovery/`)
- `sys/` - Core OS: kernel (`sys/kern`), ABI (`sys/abi`), user library (`sys/userlib`)
- `task/` - Reusable user-mode tasks (e.g., `task/jefe` supervisor, `task/net` network stack)
- `drv/` - Drivers: convention is `drv/SYSTEM-DEVICE` for driver lib, `drv/SYSTEM-DEVICE-server` for server crate
- `lib/` - Utility libraries
- `idl/` - Interface definitions in Idol format (`.idol` files)
- `chips/` - Peripheral definitions for specific MCUs
- `build/` - Build system including `xtask`
- `test/` - Test framework and test images

### Key Concepts

**App Configuration (app.toml):** Each firmware image is defined by a TOML file specifying:
- Target architecture and board
- Kernel configuration
- Task definitions with priorities, memory requirements, features
- Task slots for IPC connections
- Interrupt mappings

**Idol IPC System:** Tasks communicate via typed IPC defined in `idl/*.idol` files. The Idol compiler generates client/server stubs from these interface definitions.

**Task Slots:** Tasks declare dependencies on other tasks via `task-slots` in app.toml. These are resolved at build time to task indices.

**Jefe:** The supervisor task (`task/jefe`) manages task lifecycle and fault handling.

### Creating a New Task

1. Copy `task/template` to a new name
2. Edit its `Cargo.toml` with your package name
3. Add to workspace members in root `Cargo.toml`
4. Add task entry to an `app.toml` file
5. Build with `cargo xtask build`

### Rust Toolchain

Uses nightly Rust (pinned in `rust-toolchain.toml`). The toolchain and targets are automatically installed by rustup when building.

## External Tools

- **Humility:** Hubris debugger - install via `cargo install --git https://github.com/oxidecomputer/humility.git --locked humility-bin` (run from a different directory to avoid toolchain conflicts)
- **OpenOCD/pyOCD:** For flashing and debugging
- **arm-none-eabi-gdb:** For GDB debugging sessions
