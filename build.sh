#!/bin/sh
wasm-pack build --target web --release
node patch_pkg_json.js