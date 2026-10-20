# Changelog

All notable changes, one entry per ticket. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- **NX-01** Spike: MuJoCo WASM steps the arm in Node and in the browser at about 5× real
  time for two arms with 12 node instances. `nexus-wasm` runs one node per WebAssembly
  instance over a C ABI; `WasmNode` loader with isolation tests; `npm run bench:step`;
  spike page in `web/`; ADR-0002. Toolchain moved to Rust 1.99.
- **NX-00** Repository scaffold: Cargo and npm workspaces, `nexus-proto` and `nexus-node`
  crate skeletons, `sim` harness skeleton, formatting, linting and test tooling behind
  `npm run check`, ticket files and generated board, ADR-0001.
