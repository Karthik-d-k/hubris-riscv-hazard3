file target/demo-rp2350-hazard3/dist/kernel

target extended-remote localhost:3333

add-symbol-file target/demo-rp2350-hazard3/dist/minisuper.elf
add-symbol-file target/demo-rp2350-hazard3/dist/blinky.elf
add-symbol-file target/demo-rp2350-hazard3/dist/idle.elf

set substitute-path /git C:/Users/adt8kor/.cargo/git/checkouts
set substitute-path /crates.io C:/Users/adt8kor/.cargo/registry/src/github.com-1ecc6299db9ec823
set substitute-path /crates.io C:/Users/adt8kor/.cargo/registry/src/index.crates.io-6f17d22bba15001f
set substitute-path /hubris C:/Users/adt8kor/wws/riscv/hubris-riscv-hazard3

set pagination off

monitor arm semihosting enable

source app/demo-hazard3/gdbinit
