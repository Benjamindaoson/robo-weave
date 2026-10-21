#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [ -e .venv-arm ]; then
  echo 'Existing .venv-arm preserved; inspect it before changing dependencies.'
  .venv-arm/bin/python -c 'import numpy,mani_skill,mplib; print("NumPy",numpy.__version__)'
  exit 0
fi
"${ARM_PYTHON:-python3.10}" -m venv .venv-arm
if [ -n "${MANISKILL_SITE_PACKAGES:-}" ]; then
  # Reuse reviewed packages read-only; keep the incompatible NumPy override local.
  test -d "$MANISKILL_SITE_PACKAGES/mani_skill"
  .venv-arm/bin/pip install numpy==1.26.4
  SITE=$(.venv-arm/bin/python -c 'import site;print(site.getsitepackages()[0])')
  printf '%s\n' "$MANISKILL_SITE_PACKAGES" > "$SITE/reused-maniskill.pth"
else
  .venv-arm/bin/pip install -r requirements-arm.txt
fi
.venv-arm/bin/python -c 'import numpy,mani_skill,mplib,torch; print("canary",numpy.__version__,torch.__version__)'
