#!/usr/bin/env bash
set -euo pipefail

cd ~/icecream
source ./icecream/bin/activate
python -m arm_control.main
