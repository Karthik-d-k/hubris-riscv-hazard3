//! Build script for Layer 0
//!
//! This script tells cargo where to find the linker scripts.
//! We provide our own link.x (similar to Hubris's kernel-link.x) that
//! includes support for RP2350's IMAGE_DEF block.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Get the output directory
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Tell the linker where to find linker scripts
    println!("cargo:rustc-link-search={}", out_dir.display());

    // Copy link.x to OUT_DIR
    File::create(out_dir.join("link.x"))
        .unwrap()
        .write_all(include_bytes!("link.x"))
        .unwrap();

    // Copy memory.x to OUT_DIR (link.x includes this)
    File::create(out_dir.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // Rebuild if these files change
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
