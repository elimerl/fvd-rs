#!/bin/sh
wasm-pack build --target web --release
node patch.js
cd pkg && pnpm pack