use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    let mut file = File::create(out.join("memory.x")).unwrap();

    write!(
        file,
        r#"MEMORY
{{
FLASH : ORIGIN = 0x08000000, LENGTH = 32K
RAM : ORIGIN = 0x20000000, LENGTH = 6K
}}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
"#
    )
    .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=.cargo/config.toml");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
