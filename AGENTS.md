# Repository Guidelines

## Project Structure & Module Organization
- `src/main.rs` boots the native eframe app, passing optional STEP paths from CLI; `src/lib.rs` re-exports the app; `src/app.rs` owns UI state, parsing, and placeholder rendering; `src/renderer.rs` holds three-d camera/mesh helpers.
- `assets/` contains icons, manifest, and `sw.js` for the web build; keep cache names in sync when renaming the crate.
- `index.html` and `Trunk.toml` configure the WASM build; `examples/parse_test.rs` is a CLI parser smoke test.
- `check.sh` runs the full CI-like suite locally. Avoid editing `target/` or other generated outputs.

## Build, Test, and Development Commands
- `cargo run --release [<path/to/file.step>]` — launch the native viewer (you can pass a STEP path).
- `cargo run --example parse_test` — parse the sample STEP file without the GUI.
- `./check.sh` — fmt, clippy (warnings-as-errors), tests, wasm check, and a Trunk build; fastest way to match CI.
- Web: `rustup target add wasm32-unknown-unknown && cargo install --locked trunk` once, then `trunk serve` for live reload at `http://127.0.0.1:8080/index.html#dev`; `trunk build --release` for deployable `dist/`.

## Coding Style & Naming Conventions
- Rust 2024 edition; prefer idiomatic snake_case for files/modules and UpperCamelCase for types (e.g., `StepViewerApp`, `StepRenderer`).
- Run `cargo fmt` before committing; `cargo clippy --all-targets --all-features -- -D warnings` should stay clean.
- Keep UI logic in `app.rs` and rendering helpers in `renderer.rs`; small helper functions are favored over long blocks in `update`.

## Testing Guidelines
- Add unit tests near the code they cover or in `tests/` for integration; include doc tests for examples.
- Run `cargo test --all-targets --all-features` and `cargo test --doc` (already in `check.sh`).
- For parser changes, update or extend `examples/parse_test.rs`; keep sample STEP fixtures small and committed.

## Commit & Pull Request Guidelines
- Prefer concise, imperative commits (`Add STEP entity legend`, `Fix trunk build warnings`); keep scope tight.
- In PRs, describe the user-visible change, how it was tested (commands), and attach screenshots/GIFs for UI updates.
- Link related issues when available; note any platform-specific considerations (native vs. wasm) and asset changes that affect caching.
