# use PowerShell instead of sh:
set shell := ["powershell.exe", "-c"]

alias b := build
alias r := reboot
alias f := flash
alias g := gdb
alias d := dump
alias e := entry-point

default:
    @just --list

build:
    cargo xtask dist .\app\demo-rp2350-hazard3\app.toml

reboot:
    picotool reboot -u -c riscv

flash:
    openocd -f chips\rp2350-hazard3\openocd.cfg -c "program target/demo-rp2350-hazard3/dist/kernel verify"

entry-point:
    @echo ("IDLE   Entry Point: " + ((riscv32-unknown-elf-readelf.exe -h target\demo-rp2350-hazard3\dist\idle.elf | Select-String "Entry point address") -split ":")[1].Trim())
    @echo ("SUPER  Entry Point: " + ((riscv32-unknown-elf-readelf.exe -h target\demo-rp2350-hazard3\dist\minisuper.elf | Select-String "Entry point address") -split ":")[1].Trim())
    @echo ("BLINKY Entry Point: " + ((riscv32-unknown-elf-readelf.exe -h target\demo-rp2350-hazard3\dist\blinky.elf | Select-String "Entry point address") -split ":")[1].Trim())
    @echo ("KERNEL Entry Point: " + ((riscv32-unknown-elf-readelf.exe -h target\demo-rp2350-hazard3\dist\kernel | Select-String "Entry point address") -split ":")[1].Trim())

gdb: entry-point
    riscv32-unknown-elf-gdb.exe -x chips\rp2350-hazard3\openocd.gdb

dump:
    riscv32-unknown-elf-objdump.exe -d target\demo-rp2350-hazard3\dist\idle.elf -S > idle-dump.txt
    riscv32-unknown-elf-objdump.exe -d target\demo-rp2350-hazard3\dist\minisuper.elf -S > minisuper-dump.txt
    riscv32-unknown-elf-objdump.exe -d target\demo-rp2350-hazard3\dist\blinky.elf -S > blinky-dump.txt
    riscv32-unknown-elf-objdump.exe -d target\demo-rp2350-hazard3\dist\kernel -S > kernel-dump.txt
