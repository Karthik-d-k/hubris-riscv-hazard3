# Hubris OS - Complete Learning Guide

## Table of Contents
1. [What is Hubris OS?](#what-is-hubris-os)
2. [Prerequisites](#prerequisites)
3. [Documentation Resources](#documentation-resources)
4. [Learning Path](#learning-path)
5. [Where to Start Reading Code](#where-to-start-reading-code)
6. [Hands-On: Your First Steps](#hands-on-your-first-steps)
7. [Understanding Core Concepts](#understanding-core-concepts)
8. [Advanced Topics](#advanced-topics)
9. [Reference Materials](#reference-materials)

---

## What is Hubris OS?

**Hubris** is a microcontroller operating environment designed for deeply-embedded systems with reliability requirements. It's specifically engineered for 32-bit microcontrollers with kiB to MiB of RAM/Flash.

### Key Characteristics

**Design Philosophy:**
- **Robustness-focused**: Memory safety, fault isolation, and holistic deployment
- **Microkernel-inspired**: Most code runs unprivileged (drivers in tasks, not kernel)
- **Real-time capable**: Strict priority scheduling, no time-slicing
- **Physically addressed**: All tasks visible in single address space for debugging
- **Fixed architecture**: No runtime task creation/destruction

**Core Features:**
- Written primarily in Rust for memory safety
- Separate compilation and memory isolation for each task
- Synchronous IPC (no message queues to size)
- Prevents fault amplification and denial-of-service scenarios
- Holistic firmware deployment (all tested together)
- Drivers live in unprivileged tasks, not the kernel
- ~2000 lines of kernel code (trusted codebase)

**Target Hardware:**
- ARM Cortex-M processors (ARMv6-M, ARMv7E-M, ARMv8-M Main-profile)
- Requires Memory Protection Unit (MPU)
- Primary targets: STM32H74/5, NXP LPC55S
- Secondary support: STM32F3/4, STM32H7B, STM32G0

---

## Prerequisites

### Required Knowledge

**1. Rust Programming (Essential)**
- Ownership, borrowing, lifetimes
- Unsafe Rust basics
- Cargo and the Rust toolchain
- Understanding of `#![no_std]` embedded Rust

**2. Embedded Systems Fundamentals (Essential)**
- ARM Cortex-M architecture basics
- Memory-mapped I/O
- Interrupts and interrupt handling
- UART, I2C, SPI protocols
- Memory protection units (MPU)

**3. Operating Systems Concepts (Recommended)**
- Process/task scheduling
- Inter-process communication (IPC)
- System calls
- Context switching
- Memory protection and isolation

**4. Real-Time Systems (Helpful)**
- Priority-based scheduling
- Worst-case execution time
- Latency vs throughput
- Priority inversion

### Required Tools

**Development Environment:**
```bash
# 1. Rust toolchain (managed by rustup)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 2. System libraries (Ubuntu/Debian)
sudo apt-get install libusb-1.0-0-dev libftdi1-dev

# 3. ARM GDB debugger
sudo apt-get install arm-none-eabi-gdb

# 4. Humility debugger (Hubris-specific)
cargo install --git https://github.com/oxidecomputer/humility.git --locked humility-bin

# 5. OpenOCD (for flashing and debugging)
sudo apt-get install openocd

# 6. Graphviz (optional, for task graphs)
sudo apt-get install graphviz
```

**Hardware (One of):**
- STM32F4 Discovery board (~$20, recommended for beginners)
- STM32H7 Nucleo board
- LPCXpresso55S69 board
- Any supported Oxide hardware

---

## Documentation Resources

### Official Documentation (Start Here)

**Online Documentation:**
- Website: https://hubris.oxide.computer/
- Reference: https://oxidecomputer.github.io/humility
- Main Repo: https://github.com/oxidecomputer/hubris

**Local Documentation Files:**

**Essential Reading (in order):**
1. `README.mkdn` - Build, development, usage instructions
2. `doc/intro.adoc` - Philosophy and key features
3. `FAQ.mkdn` - Common questions and design rationale
4. `doc/tasks.adoc` - Task model and scheduling
5. `doc/ipc.adoc` - Inter-process communication (26KB, comprehensive)
6. `doc/syscalls.adoc` - Kernel syscall interface (22KB)

**Core Reference Documents:**
- `doc/interrupts.adoc` - Interrupt handling model
- `doc/timers.adoc` - Timer multiplexing
- `doc/startup.adoc` - CPU initialization and boot
- `doc/kipc.adoc` - Kernel IPC for privileged operations
- `doc/tr1.adoc` - Task restart semantics (18KB)

**Application Development Guides:**
- `doc/guide/servers.adoc` - Server implementation patterns
- `doc/guide/drivers.adoc` - Driver architecture
- `doc/guide/supervision.adoc` - Fault supervision
- `doc/guide/caboose.adoc` - Device metadata storage

---

## Learning Path

### Phase 1: Understanding (Week 1-2)

**Goal:** Understand what Hubris is and why it's designed this way

**Reading List:**
1. Read `README.mkdn` (entire file, 724 lines)
2. Read `doc/intro.adoc` - Get the philosophy
3. Read `FAQ.mkdn` - Understand design decisions
4. Skim `doc/tasks.adoc` - Get task model overview

**Questions to Answer:**
- Why synchronous IPC instead of asynchronous?
- Why no runtime task creation?
- How does task isolation work?
- What happens when a task crashes?

### Phase 2: Building and Exploring (Week 2-3)

**Goal:** Get hands-on experience building and running Hubris

**Activities:**
```bash
# 1. Clone the repository
git clone https://github.com/oxidecomputer/hubris.git
cd hubris

# 2. Build a simple demo
cargo xtask dist app/demo-stm32f4-discovery/app.toml

# 3. Flash to hardware (if available)
cargo xtask flash app/demo-stm32f4-discovery/app.toml

# 4. Explore with Humility
cargo xtask humility app/demo-stm32f4-discovery/app.toml -- tasks
cargo xtask humility app/demo-stm32f4-discovery/app.toml -- map

# 5. Generate task graph
cargo xtask graph -o demo.dot app/demo-stm32f4-discovery/app.toml
dot -Tsvg demo.dot > demo.svg
```

**Code to Explore:**
1. `sys/kern/` - Kernel implementation
2. `sys/abi/` - System call interface
3. `sys/userlib/` - User library for tasks
4. `task/template/` - Task template
5. `app/demo-stm32f4-discovery/` - Simple application

### Phase 3: Deep Dive into Core Concepts (Week 3-5)

**Goal:** Master the fundamental mechanisms

**Study Path:**

**A. Task System**
- Read `doc/tasks.adoc` thoroughly
- Examine `sys/kern/task.rs`
- Study `task/ping/` and `task/pong/` for examples
- Understand separate compilation model

**B. IPC System**
- Read `doc/ipc.adoc` (26KB, very detailed)
- Study syscall definitions in `sys/abi/`
- Look at `sys/userlib/src/lib.rs` for user API
- Examine server examples in `drv/*/`

**C. Scheduling and Priorities**
- Understand strict priority scheduling
- Study kernel scheduler in `sys/kern/`
- Look at `app.toml` files to see priority assignments
- Understand preemption model

**D. Memory Protection**
- Study MPU configuration in kernel
- Understand task memory regions
- Look at linker scripts: `build/kernel-link.x`, `build/task-link.x`

### Phase 4: Building Your Own Task (Week 5-6)

**Goal:** Create and integrate a custom task

**Project: Simple Blinky Task**

```bash
# 1. Copy template
cp -r task/template task/my-blink

# 2. Edit Cargo.toml
# Change name to "task-my-blink"
# Change package name to "task-my-blink"

# 3. Add to root Cargo.toml
# Add "task/my-blink" to workspace.members

# 4. Implement in src/main.rs
# Use GPIO to blink an LED

# 5. Add to app.toml
[tasks.my_blink]
name = "my-blink"
priority = 5
requires = {flash = 2048, ram = 1024}
start = true

# 6. Build and test
cargo xtask build app/demo-stm32f4-discovery/app.toml my_blink
cargo xtask flash app/demo-stm32f4-discovery/app.toml
```

**Learning Objectives:**
- Task structure and lifecycle
- Using system calls
- IPC communication
- Peripheral access
- Debugging with Humility

### Phase 5: Understanding Drivers (Week 6-8)

**Goal:** Learn how to write device drivers

**Study Materials:**
- Read `doc/guide/drivers.adoc`
- Examine simple drivers:
  - `drv/stm32xx-gpio-common/` - GPIO driver
  - `drv/stm32fx-usart/` - UART driver
  - `drv/stm32h7-i2c-server/` - I2C server

**Key Concepts:**
- Driver crates vs server tasks
- Idol interface definitions (`idl/*.idol`)
- Peripheral access and safety
- Error handling and recovery
- Client-server patterns

**Project: Write a Simple Driver**
- Choose a peripheral (e.g., ADC, PWM)
- Create driver crate in `drv/`
- Define Idol interface in `idl/`
- Implement server task
- Test with client task

### Phase 6: Advanced Topics (Week 8-12)

**Goal:** Master advanced features and patterns

**Topics:**

**A. Supervision and Recovery**
- Read `doc/guide/supervision.adoc`
- Study `task/jefe/` (supervisor task)
- Understand task restart semantics (`doc/tr1.adoc`)
- Implement recovery strategies

**B. Interrupts and Timing**
- Read `doc/interrupts.adoc`
- Read `doc/timers.adoc`
- Study interrupt handling in drivers
- Timer multiplexing examples

**C. Complex Applications**
- Study `app/gimletlet/` structure
- Understand multi-board configurations
- Sequencing tasks for power management
- Thermal management patterns

**D. Testing**
- Study `test/test-suite/` structure
- Read test framework in `test/test-runner/`
- Run tests: `cargo xtask test test/tests-stm32fx/app.toml`
- Write tests for your components

**E. Build System**
- Understand `xtask` architecture (`build/xtask/`)
- Study `app.toml` configuration format
- Learn about linker scripts and memory layout
- Explore code generation tools

---

## Where to Start Reading Code

### Recommended Reading Order

#### 1. Start Simple: Task Template
**File:** `task/template/src/main.rs`
**Why:** Shows basic task structure
**Key points:**
- Task entry point
- System call usage
- Basic loop structure

#### 2. Minimal Tasks: Ping and Pong
**Files:**
- `task/ping/src/main.rs`
- `task/pong/src/main.rs`

**Why:** Demonstrates IPC between tasks
**Key points:**
- SEND operation
- RECV operation
- Message passing

#### 3. System Call Interface
**File:** `sys/abi/src/lib.rs`
**Why:** Defines the kernel API
**Key points:**
- Syscall numbers
- Operation codes
- Error codes

#### 4. User Library
**File:** `sys/userlib/src/lib.rs`
**Why:** Rust wrappers around syscalls
**Key points:**
- Safe syscall wrappers
- Helper functions
- Common patterns

#### 5. Simple Driver: GPIO
**Directory:** `drv/stm32xx-gpio-common/`
**Why:** Shows driver patterns
**Key points:**
- Peripheral access
- Register manipulation
- Safety considerations

#### 6. Kernel Entry Points
**Files:**
- `sys/kern/src/task.rs` - Task management
- `sys/kern/src/ipc.rs` - IPC implementation
- `sys/kern/src/startup.rs` - Boot sequence

**Why:** Understand kernel internals
**Warning:** More complex, save for later

#### 7. Application Structure
**File:** `app/demo-stm32f4-discovery/app.toml`
**Why:** See how everything connects
**Key points:**
- Task definitions
- Memory allocation
- Interrupt routing
- Dependencies

### Code Navigation Tips

**By Directory:**
```
sys/
├── kern/          # START HERE for kernel
├── abi/           # START HERE for syscall interface
├── userlib/       # START HERE for task programming
└── ...

task/
├── template/      # START HERE for task structure
├── ping/          # Simple IPC example
├── pong/          # Simple IPC example
└── jefe/          # Complex supervisor (later)

drv/
├── stm32xx-gpio-common/    # Simple driver example
├── stm32fx-usart/          # UART driver
└── [100+ more drivers]     # Reference material

app/
├── demo-stm32f4-discovery/ # START HERE for simple app
├── gimletlet/              # Complex production app (later)
└── ...
```

**Reading Strategy:**
1. Top-down: Start with high-level docs, then code
2. Bottom-up: Start with simple tasks, build understanding
3. Follow the data: Trace a message from send to recv
4. Use search: `git grep` for function definitions
5. Generate graphs: Visualize task relationships

---

## Hands-On: Your First Steps

### Exercise 1: Build and Flash
```bash
# Build the STM32F4 demo
cargo xtask dist app/demo-stm32f4-discovery/app.toml

# Flash to board
cargo xtask flash app/demo-stm32f4-discovery/app.toml

# Observe running tasks
cargo xtask humility app/demo-stm32f4-discovery/app.toml -- tasks
```

### Exercise 2: Explore with Humility
```bash
# Show task list
humility -a target/demo-stm32f4-discovery/dist/build-demo-stm32f4-discovery.zip tasks

# Show memory map
humility -a target/demo-stm32f4-discovery/dist/build-demo-stm32f4-discovery.zip map

# Dump task state
humility -a target/demo-stm32f4-discovery/dist/build-demo-stm32f4-discovery.zip dump
```

### Exercise 3: Modify a Task
```bash
# 1. Edit task/user_leds/src/main.rs
# 2. Change LED blink pattern
# 3. Rebuild just that task
cargo xtask build app/demo-stm32f4-discovery/app.toml user_leds

# 4. Flash and observe
cargo xtask flash app/demo-stm32f4-discovery/app.toml
```

### Exercise 4: Add Your Own Task
```bash
# 1. Copy template
cp -r task/template task/hello

# 2. Edit Cargo.toml and src/main.rs

# 3. Add to root Cargo.toml workspace

# 4. Add to app.toml:
[tasks.hello]
name = "hello"
priority = 3
requires = {flash = 1024, ram = 1024}
start = true

# 5. Build
cargo xtask build app/demo-stm32f4-discovery/app.toml hello
```

### Exercise 5: Debug with GDB
```bash
# Start GDB session
cargo xtask gdb app/demo-stm32f4-discovery/app.toml -- --run-openocd

# In GDB:
(gdb) break main
(gdb) continue
(gdb) backtrace
(gdb) info registers
```

---

## Understanding Core Concepts

### 1. Task Model

**What is a Task?**
- Separately compiled program
- Runs in unprivileged mode
- Memory isolated via MPU
- Has fixed priority (0-255)
- Cannot create other tasks
- Can be restarted independently

**Task Lifecycle:**
```
┌─────────┐
│  Born   │
└────┬────┘
     │
     v
┌─────────────┐    Fault    ┌──────────┐
│   Healthy   ├────────────>│  Faulted │
└──┬───────┬──┘             └────┬─────┘
   │       │                     │
   │       └─────────────────────┘
   │              Restart
   │
   └──> Running/Blocked/Ready states
```

**Task States:**
- **Runnable**: Ready to execute
- **InRecv**: Blocked waiting for message
- **InSend**: Blocked sending message
- **InReply**: Blocked waiting for reply
- **Faulted**: Crashed, awaiting restart

### 2. IPC System

**Message Types:**
- **SEND**: Synchronous call with reply
- **RECV**: Receive incoming message
- **REPLY**: Send reply to waiting client
- **NOTIFY**: Asynchronous notification

**Call Flow:**
```
Client Task          Server Task
     |                    |
SEND ├───message────────> RECV
     │                    │
     │              (process)
     │                    │
     ├<────reply──────────┤ REPLY
 return                   │
                       RECV (again)
```

**Key Properties:**
- Synchronous (caller blocks)
- No message queues
- Zero-copy via leases
- Priority-ordered delivery

### 3. Scheduling

**Priority Model:**
- Strict priority (0 = highest)
- No time-slicing
- Immediate preemption
- Each task has unique priority (recommended)

**Scheduling Decisions:**
1. Highest priority runnable task executes
2. Equal priority: first to block releases CPU
3. Lower priority: only runs when higher tasks block

### 4. Memory Protection

**Memory Regions:**
- **Flash**: Code and constants (read-only)
- **RAM**: Data and stack (read-write)
- **Peripherals**: Memory-mapped I/O (task-specific)

**MPU Configuration:**
- Kernel configures MPU for each task
- Task can only access its regions
- Violations cause immediate fault
- Fault notification to supervisor

---

## Advanced Topics

### System Call Interface

**Core Syscalls:**
```rust
// Send message and wait for reply
sys_send(task_id, operation, message, reply_buffer, leases)

// Receive incoming message
sys_recv(buffer, notification_mask)

// Reply to waiting sender
sys_reply(caller_id, code, message)

// Set timer
sys_set_timer(deadline, notification_bits)

// Wait for notification
sys_recv_notification(mask)
```

### Idol Interface Language

**Purpose:** Define IPC interfaces between tasks

**Example IDL:**
```idol
Interface(
    name: "Blink",
    ops: {
        "on": (
            args: {},
            reply: Result<(), LedError>,
        ),
        "off": (
            args: {},
            reply: Result<(), LedError>,
        ),
    },
)
```

### Build System Deep Dive

**xtask Architecture:**
- Custom Cargo subcommand
- Handles multi-architecture builds
- Generates build archives
- Integrates with Humility

**Key Commands:**
```bash
cargo xtask dist <APP.TOML>     # Full build
cargo xtask build <APP.TOML> <TASKS>  # Incremental
cargo xtask clippy <APP.TOML>   # Lint
cargo xtask test <TEST.TOML>    # Run tests
cargo xtask graph <APP.TOML>    # Generate graph
cargo xtask lsp <FILE>          # LSP config
```

---

## Reference Materials

### Quick Reference

**Syscall Numbers:**
- See `sys/abi/src/lib.rs`

**Error Codes:**
- `UsageError` - Bad syscall parameters
- `NoSuchTask` - Invalid task ID
- `Dead` - Target task is dead
- `Restart` - Task was restarted

**Task Priorities:**
- 0 = Highest (most important)
- 255 = Lowest (least important)
- Idle task typically at lowest priority

### Keyboard Shortcuts and Commands

**Humility Commands:**
```bash
humility tasks          # List tasks
humility map            # Memory map
humility dump           # Full dump
humility readvar <VAR>  # Read variable
humility gpio           # GPIO state
humility i2c            # I2C operations
humility probe          # Probe info
```

**GDB Commands:**
```gdb
info threads            # List tasks
thread <N>              # Switch task
backtrace               # Stack trace
print <expr>            # Evaluate
x/<fmt> <addr>          # Examine memory
```

### Common Patterns

**Server Pattern:**
```rust
loop {
    let (msg, caller) = sys_recv(&mut buffer, NOTIFICATION_MASK)?;
    match msg.operation {
        Op::Foo => {
            let result = do_foo();
            sys_reply(caller, 0, &result);
        }
    }
}
```

**Client Pattern:**
```rust
let result = send(
    server_task_id,
    Operation::Foo,
    &request,
    &mut response,
);
```

**Notification Pattern:**
```rust
// Sender
sys_post(target_task, notification_bits);

// Receiver
let notifications = sys_recv_notification(mask)?;
if notifications & BIT_FOO != 0 {
    handle_foo();
}
```

### Troubleshooting

**Build Errors:**
```bash
# Clean build
cargo clean
cargo xtask dist <APP.TOML>

# Check toolchain
rustup show

# Update xtask
cargo build -p xtask
```

**Flash Errors:**
```bash
# Check probe
humility probe

# Update ST-Link firmware
# (see README.mkdn section)

# Try OpenOCD directly
openocd -f <config>.cfg
```

**Runtime Errors:**
```bash
# Dump state
humility -d dump

# Check task state
humility tasks

# View logs
humility readvar <task>::LOG
```

---

## Summary: Your Learning Journey

### Milestone Checklist

- [ ] Phase 1: Understand Hubris philosophy and design (Week 1-2)
- [ ] Phase 2: Build and run demo application (Week 2-3)
- [ ] Phase 3: Master core concepts (tasks, IPC, scheduling) (Week 3-5)
- [ ] Phase 4: Create custom task (Week 5-6)
- [ ] Phase 5: Write device driver (Week 6-8)
- [ ] Phase 6: Advanced topics and production patterns (Week 8-12)

### Continuous Learning

**Stay Updated:**
- Follow GitHub repository: https://github.com/oxidecomputer/hubris
- Join discussions and PRs
- Read commit messages for design insights

**Practice Projects:**
1. LED blinker with button input
2. UART echo server
3. I2C sensor driver
4. Multi-task communication demo
5. Fault recovery demonstration
6. Real-time control application

**Community:**
- GitHub Issues for questions
- Pull requests for contributions
- Study existing PRs for patterns

---

## Conclusion

Hubris is a unique OS that requires understanding both its philosophy and implementation. Take your time with each phase, focus on understanding "why" before "how", and gradually build up your knowledge through hands-on experimentation.

The key to mastering Hubris is:
1. **Understand the philosophy** - Why synchronous? Why isolated?
2. **Read the documentation** - It's excellent and thorough
3. **Experiment hands-on** - Build, flash, debug, repeat
4. **Study existing code** - Learn from production examples
5. **Write your own tasks** - Best way to truly understand

Good luck on your Hubris learning journey!
