# Repository Guidelines

## Project Structure & Module Organization

This repository is a Python simulation and evaluation workspace for pure-vision PNG guidance with AirSim/PX4 experiments. Core reusable code lives in `vision_guidance/`, including geometry, LOS filtering, TTC estimation, AirSim adapters, and detector integration. Scenario runners and report generators live in `examples/`; shell wrappers in the repository root launch common AirSim Blocks, PX4 SITL/HIL, strapdown, and YOLO batch workflows. Unit tests are in `tests/` and mirror module names with `test_*.py`. Configuration JSON files are in `config/`; PX4 utility scripts are in `tools/`. Generated logs, datasets, videos, and local virtual environments are ignored by git.

## Build, Test, and Development Commands

- `python3 -m unittest discover -s tests -v`: run the unit test suite.
- `python3 examples/run_synthetic.py`: run the synthetic guidance example.
- `python3 examples/run_airsim_blocks.py --duration-s 30`: collect AirSim Blocks detection/evaluation logs without vehicle motion.
- `python3 examples/run_airsim_strapdown_vision_png.py --enable-motion --intruder-speed 5 --speed-ratio 2`: run the fixed-camera AirSim validation path.
- `./run_px4_sitl.sh` or `./run_blocks_px4_sitl.sh`: start PX4 SITL / AirSim Blocks workflows.

Install optional YOLO dependencies only when needed: `python3 -m pip install torch ultralytics lap opencv-contrib-python`.

## Coding Style & Naming Conventions

Use Python 3 with 4-space indentation, `snake_case` functions and variables, and `PascalCase` classes/dataclasses. Keep numerical code explicit and prefer NumPy arrays for vector and matrix operations. Preserve existing type hints and concise docstrings for coordinate-frame or convention-sensitive functions. Avoid hidden fallbacks in simulation paths; fail fast when required models, class IDs, or external services are missing.

## Testing Guidelines

Tests use the standard `unittest` framework. Add new tests under `tests/test_<module>.py`, with methods named `test_<behavior>`. Prefer deterministic tests around math, filtering, gating, and adapter contracts. For AirSim/PX4-dependent changes, keep unit tests independent of the simulator and document any manual validation command plus generated `logs/` artifacts in the PR.

## Commit & Pull Request Guidelines

Recent commits use short imperative subjects such as `Add ...`, `Support ...`, and `Document ...`. Follow that style and keep each commit focused on one behavior, report, or workflow. Pull requests should include a concise description, commands run, relevant simulator settings or config files, linked issues when applicable, and screenshots or plots for report/trajectory changes. Do not commit generated `logs/`, `datasets/`, `.venv/`, or large transient media.

## Security & Configuration Tips

Keep local secrets and machine-specific paths in `.env` or shell configuration, not in tracked scripts. Review `config/*.json` before sharing simulator settings, especially vehicle names, ports, and host-specific paths.
