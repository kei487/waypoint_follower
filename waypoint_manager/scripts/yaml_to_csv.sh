#!/usr/bin/env bash
# Convert legacy waypoint_manager YAML configs to the shared CSV format.
#
# Usage:
#   yaml_to_csv.sh INPUT.yaml [OUTPUT.csv]
#   yaml_to_csv.sh -r 0.5 INPUT1.yaml INPUT2.yaml ...
#   yaml_to_csv.sh -o out.csv INPUT.yaml
#
# Options:
#   -r, --default-radius RADIUS   Fallback radius when YAML omits it (default: 0.5)
#   -o, --output PATH             Output CSV path (single input only)
#   -h, --help                    Show this help

set -euo pipefail

DEFAULT_RADIUS="0.5"
OUTPUT_PATH=""
INPUTS=()

usage() {
  sed -n '2,14p' "$0" | sed 's/^# \?//'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -r|--default-radius)
      DEFAULT_RADIUS="${2:?--default-radius requires a value}"
      shift 2
      ;;
    -o|--output)
      OUTPUT_PATH="${2:?--output requires a path}"
      shift 2
      ;;
    --)
      shift
      INPUTS+=("$@")
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
    *)
      INPUTS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#INPUTS[@]} -eq 0 ]]; then
  echo "Error: at least one INPUT.yaml is required." >&2
  usage >&2
  exit 1
fi

if [[ -n "${OUTPUT_PATH}" && ${#INPUTS[@]} -ne 1 ]]; then
  echo "Error: --output can only be used with a single input file." >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "Error: python3 is required." >&2
  exit 1
fi

convert_one() {
  local input="$1"
  local output="$2"

  if [[ ! -f "${input}" ]]; then
    echo "Error: input not found: ${input}" >&2
    return 1
  fi

  DEFAULT_RADIUS="${DEFAULT_RADIUS}" INPUT_YAML="${input}" OUTPUT_CSV="${output}" python3 - <<'PY'
import math
import os
import sys

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required (pip install pyyaml / apt install python3-yaml).", file=sys.stderr)
    sys.exit(1)

input_path = os.environ["INPUT_YAML"]
output_path = os.environ["OUTPUT_CSV"]
default_radius = float(os.environ["DEFAULT_RADIUS"])

with open(input_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

if not isinstance(data, dict) or "waypoints" not in data:
    print(f"Error: missing top-level 'waypoints' in {input_path}", file=sys.stderr)
    sys.exit(1)

waypoints = data["waypoints"]
if waypoints is None:
    waypoints = []
if not isinstance(waypoints, list):
    print(f"Error: 'waypoints' must be a list in {input_path}", file=sys.stderr)
    sys.exit(1)

rows = []
for idx, wp in enumerate(waypoints):
    if not isinstance(wp, dict):
        print(f"Warning: skipping non-map waypoint entry at index {idx}", file=sys.stderr)
        continue

    wp_id = wp.get("id", idx)
    position = wp.get("position") or {}
    euler = wp.get("euler_angle") or {}

    x = float(position.get("x", 0.0))
    y = float(position.get("y", 0.0))
    z = float(position.get("z", 0.0))
    yaw = float(euler.get("z", 0.0))

    # Match former manager conversion: yaw -> quaternion (z, w only)
    rot_x = 0.0
    rot_y = 0.0
    rot_z = math.sin(yaw / 2.0)
    rot_w = math.cos(yaw / 2.0)

    radius = default_radius
    functions = wp.get("functions")
    if isinstance(functions, list):
        for fn in functions:
            if not isinstance(fn, dict):
                continue
            if fn.get("function") == "variable_waypoint_radius" and fn.get("waypoint_radius") is not None:
                radius = float(fn["waypoint_radius"])
                break

    robot_wait = bool(wp.get("robot_wait", False))
    command = wp.get("command", "")
    if command is None:
        command = ""
    command = str(command)

    rows.append(
        (
            wp_id,
            x,
            y,
            z,
            rot_x,
            rot_y,
            rot_z,
            rot_w,
            radius,
            "true" if robot_wait else "false",
            command,
        )
    )

os.makedirs(os.path.dirname(os.path.abspath(output_path)) or ".", exist_ok=True)
with open(output_path, "w", encoding="utf-8", newline="") as f:
    f.write("id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,waypoint_radius,robot_wait,command,\n")
    for row in rows:
        # Keep command in a single cell; trailing comma matches editor CSV style
        f.write(
            "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},\n".format(*row)
        )

print(f"Converted {len(rows)} waypoints: {input_path} -> {output_path}")
PY
}

for input in "${INPUTS[@]}"; do
  if [[ -n "${OUTPUT_PATH}" ]]; then
    output="${OUTPUT_PATH}"
  else
    if [[ "${input}" == *.yaml || "${input}" == *.yml ]]; then
      output="${input%.*}.csv"
    else
      output="${input}.csv"
    fi
  fi
  convert_one "${input}" "${output}"
done
