from __future__ import annotations

import argparse
import subprocess
import sys
import zipfile
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {".venv", "runtime", "outputs", "__pycache__", ".pytest_cache"}


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate and create the Windows simulation package ZIP.")
    parser.add_argument("--output", type=Path, default=PACKAGE_ROOT.with_suffix(".zip"))
    args = parser.parse_args()
    subprocess.run([sys.executable, str(PACKAGE_ROOT / "tools" / "check_package.py"), "--write-manifest"], check=True)
    output = args.output.resolve()
    if output.exists():
        output.unlink()
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in sorted(PACKAGE_ROOT.rglob("*")):
            if not path.is_file():
                continue
            relative = path.relative_to(PACKAGE_ROOT)
            if any(part in EXCLUDED_PARTS for part in relative.parts) or path.resolve() == output:
                continue
            archive.write(path, (Path(PACKAGE_ROOT.name) / relative).as_posix())
    print(f"release_zip={output}")


if __name__ == "__main__":
    main()
