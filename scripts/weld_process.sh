#!/usr/bin/env bash
set -euo pipefail

# weld_process.sh
# Simple welding prep and image-processing pipeline.
# - Sets default joint geometry
# - Uses a sample image if none is provided
# - Performs basic edge detection and seam estimation
# - Produces outputs in an output directory (images: PGM/PPM, data: CSV/JSON)
#
# Usage:
#   scripts/weld_process.sh \
#     [-t JOINT_TYPE] [-o ROOT_OPENING_MM] [-a BEVEL_ANGLE_DEG] [-l LAND_THICKNESS_MM] \
#     [-r TORCH_ANGLE_DEG] [-i IMAGE_PATH] [-d OUTPUT_DIR] [-W WIDTH] [-H HEIGHT] [-s SEAM_DIRECTION]
#
# Options:
#   -t  Joint type (default: v-groove)
#   -o  Root opening in mm (default: 2.0)
#   -a  Bevel angle in degrees (default: 30.0)
#   -l  Land thickness in mm (default: 1.0)
#   -r  Torch angle in degrees (default: 15.0)
#   -i  Input image path (optional; supports ASCII PGM/PPM). If omitted, generate sample
#   -d  Output directory (default: ./weld_output)
#   -W  Sample image width when generating (default: 640)
#   -H  Sample image height when generating (default: 480)
#   -s  Seam direction: vertical|horizontal (default: vertical)
#   -h  Show help
#
# Notes:
# - This script prefers pure-Python (no external libs). It writes ASCII PGM/PPM images.
# - If you supply an image, only ASCII PGM (P2) and PPM (P3) are supported for reading.

JOINT_TYPE="v-groove"
ROOT_OPENING="2.0"
BEVEL_ANGLE="30.0"
LAND_THICKNESS="1.0"
TORCH_ANGLE="15.0"
IMAGE_PATH=""
OUTPUT_DIR="./weld_output"
IMAGE_WIDTH="640"
IMAGE_HEIGHT="480"
SEAM_DIRECTION="vertical"

usage() {
  cat <<USAGE
Usage: $0 [-t JOINT_TYPE] [-o ROOT_OPENING_MM] [-a BEVEL_ANGLE_DEG] [-l LAND_THICKNESS_MM] \
          [-r TORCH_ANGLE_DEG] [-i IMAGE_PATH] [-d OUTPUT_DIR] [-W WIDTH] [-H HEIGHT] [-s SEAM_DIRECTION]

Defaults:
  joint-type=$JOINT_TYPE, root-opening=$ROOT_OPENING mm, bevel-angle=$BEVEL_ANGLE deg,
  land-thickness=$LAND_THICKNESS mm, torch-angle=$TORCH_ANGLE deg, output-dir=$OUTPUT_DIR,
  sample-size=${IMAGE_WIDTH}x${IMAGE_HEIGHT}, seam-direction=$SEAM_DIRECTION

Examples:
  $0                               # generate sample, process, write outputs
  $0 -i /path/to/input.pgm         # process provided ASCII PGM image
  $0 -t butt -o 1.0 -a 0 -l 0      # override joint geometry
USAGE
}

while getopts ":t:o:a:l:r:i:d:W:H:s:h" opt; do
  case $opt in
    t) JOINT_TYPE="$OPTARG" ;;
    o) ROOT_OPENING="$OPTARG" ;;
    a) BEVEL_ANGLE="$OPTARG" ;;
    l) LAND_THICKNESS="$OPTARG" ;;
    r) TORCH_ANGLE="$OPTARG" ;;
    i) IMAGE_PATH="$OPTARG" ;;
    d) OUTPUT_DIR="$OPTARG" ;;
    W) IMAGE_WIDTH="$OPTARG" ;;
    H) IMAGE_HEIGHT="$OPTARG" ;;
    s) SEAM_DIRECTION="$OPTARG" ;;
    h) usage; exit 0 ;;
    :) echo "Option -$OPTARG requires an argument" >&2; usage; exit 2 ;;
    \?) echo "Unknown option -$OPTARG" >&2; usage; exit 2 ;;
  esac
done

# Ensure output directory exists and is absolute for clarity
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR_ABS=$(cd "$OUTPUT_DIR" && pwd)

# Export parameters for the Python block
export WELD_JOINT_TYPE="$JOINT_TYPE"
export WELD_ROOT_OPENING_MM="$ROOT_OPENING"
export WELD_BEVEL_ANGLE_DEG="$BEVEL_ANGLE"
export WELD_LAND_THICKNESS_MM="$LAND_THICKNESS"
export WELD_TORCH_ANGLE_DEG="$TORCH_ANGLE"
export WELD_IMAGE_PATH="${IMAGE_PATH:-}"
export WELD_OUTPUT_DIR="$OUTPUT_DIR_ABS"
export WELD_IMAGE_WIDTH="$IMAGE_WIDTH"
export WELD_IMAGE_HEIGHT="$IMAGE_HEIGHT"
export WELD_SEAM_DIRECTION="$SEAM_DIRECTION"

python3 - <<'PY'
import os, sys, json, math, random
from pathlib import Path

# Read environment
joint_type = os.environ.get('WELD_JOINT_TYPE', 'v-groove')
root_opening_mm = float(os.environ.get('WELD_ROOT_OPENING_MM', '2.0'))
bevel_angle_deg = float(os.environ.get('WELD_BEVEL_ANGLE_DEG', '30.0'))
land_thickness_mm = float(os.environ.get('WELD_LAND_THICKNESS_MM', '1.0'))
torch_angle_deg = float(os.environ.get('WELD_TORCH_ANGLE_DEG', '15.0'))
input_image_path = os.environ.get('WELD_IMAGE_PATH') or ''
output_dir = Path(os.environ.get('WELD_OUTPUT_DIR', './weld_output'))
width = int(os.environ.get('WELD_IMAGE_WIDTH', '640'))
height = int(os.environ.get('WELD_IMAGE_HEIGHT', '480'))
seam_direction = os.environ.get('WELD_SEAM_DIRECTION', 'vertical')

output_dir.mkdir(parents=True, exist_ok=True)

# --- Simple PNM (ASCII) IO helpers ---

def _pnm_tokens(f):
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        for tok in line.split():
            if tok.startswith('#'):
                break
            yield tok

def read_pnm_ascii(path: Path):
    if not path.exists():
        return None
    with path.open('r', encoding='ascii', errors='ignore') as f:
        header = f.readline().strip()
        if header not in ('P2', 'P3'):
            return None
        toks = _pnm_tokens(f)
        try:
            w = int(next(toks)); h = int(next(toks)); maxv = int(next(toks))
        except Exception:
            return None
        if w <= 0 or h <= 0 or maxv <= 0:
            return None
        if header == 'P2':
            # grayscale
            img = [[0]*w for _ in range(h)]
            for y in range(h):
                for x in range(w):
                    try:
                        v = int(next(toks))
                    except StopIteration:
                        return None
                    img[y][x] = max(0, min(255, v))
            return {'mode': 'L', 'width': w, 'height': h, 'data': img}
        else:
            # RGB
            img = [[[0,0,0] for _ in range(w)] for _ in range(h)]
            for y in range(h):
                for x in range(w):
                    try:
                        r = int(next(toks)); g = int(next(toks)); b = int(next(toks))
                    except StopIteration:
                        return None
                    img[y][x] = [max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b))]
            return {'mode': 'RGB', 'width': w, 'height': h, 'data': img}

def save_pgm_ascii(path: Path, img):
    h = len(img); w = len(img[0]) if h > 0 else 0
    with path.open('w', encoding='ascii') as f:
        f.write('P2\n')
        f.write(f'{w} {h}\n255\n')
        for y in range(h):
            row = ' '.join(str(max(0, min(255, int(v)))) for v in img[y])
            f.write(row + '\n')

def save_ppm_ascii(path: Path, img_rgb):
    h = len(img_rgb); w = len(img_rgb[0]) if h > 0 else 0
    with path.open('w', encoding='ascii') as f:
        f.write('P3\n')
        f.write(f'{w} {h}\n255\n')
        for y in range(h):
            parts = []
            for x in range(w):
                r, g, b = img_rgb[y][x]
                parts.append(f'{int(r)} {int(g)} {int(b)}')
            f.write(' '.join(parts) + '\n')

# --- Image generation ---

def generate_sample(width: int, height: int, seam_direction: str) -> list:
    # Start with bright background
    img = [[220 for _ in range(width)] for _ in range(height)]
    cx = width // 2
    cy = height // 2
    amplitude = max(4, width // 16)  # mild sinusoidal wobble
    period = max(40, height // 6)
    rng = random.Random(42)

    if seam_direction == 'horizontal':
        for y in range(height):
            # y path with wobble in x
            x_center = cx + int(amplitude * math.sin(2 * math.pi * y / period))
            yy = cy
            thickness = 3
            # Draw a horizontal dark seam line
            for x in range(max(0, x_center - width//2), min(width, x_center + width//2)):
                for t in range(-thickness//2, thickness//2 + 1):
                    y_idx = yy + t
                    if 0 <= y_idx < height:
                        img[y_idx][x] = max(0, img[y_idx][x] - 160)
    else:
        # vertical seam with slight sinusoidal deviation
        for y in range(height):
            x_center = cx + int(amplitude * math.sin(2 * math.pi * y / period))
            thickness = 3
            for t in range(-thickness//2, thickness//2 + 1):
                x_idx = x_center + t
                if 0 <= x_idx < width:
                    img[y][x_idx] = max(0, img[y][x_idx] - 160)

    # Add mild noise
    for y in range(height):
        for x in range(width):
            n = int(rng.gauss(0, 3))
            v = img[y][x] + n
            img[y][x] = max(0, min(255, v))
    return img

# --- Edge detection (Sobel) ---
Gx = [[-1, 0, 1],
      [-2, 0, 2],
      [-1, 0, 1]]
Gy = [[ 1, 2, 1],
      [ 0, 0, 0],
      [-1,-2,-1]]

def sobel_edges(img):
    h = len(img); w = len(img[0]) if h > 0 else 0
    edges = [[0 for _ in range(w)] for _ in range(h)]
    for y in range(1, h-1):
        for x in range(1, w-1):
            sx = 0; sy = 0
            for j in range(-1, 2):
                for i in range(-1, 2):
                    v = img[y+j][x+i]
                    sx += v * Gx[j+1][i+1]
                    sy += v * Gy[j+1][i+1]
            mag = int(min(255, (sx*sx + sy*sy) ** 0.5))
            edges[y][x] = mag
    return edges

# --- Seam estimation ---

def estimate_seam_points(img, seam_direction: str):
    h = len(img); w = len(img[0]) if h > 0 else 0
    points = []
    if seam_direction == 'horizontal':
        # Find darkest pixel per column
        for x in range(w):
            min_v = 1e9; min_y = 0
            for y in range(h):
                v = img[y][x]
                if v < min_v:
                    min_v = v; min_y = y
            points.append((x, min_y))
    else:
        # vertical: find darkest per row
        for y in range(h):
            min_v = 1e9; min_x = 0
            for x in range(w):
                v = img[y][x]
                if v < min_v:
                    min_v = v; min_x = x
            points.append((min_x, y))
    return points


def estimate_line_angle_deg(points, seam_direction: str):
    # Simple linear regression to compute angle
    n = len(points)
    if n < 2:
        return 0.0
    if seam_direction == 'horizontal':
        # fit y = m*x + b
        sumx = sum(p[0] for p in points)
        sumy = sum(p[1] for p in points)
        sumxx = sum(p[0]*p[0] for p in points)
        sumxy = sum(p[0]*p[1] for p in points)
        denom = (n*sumxx - sumx*sumx)
        if denom == 0:
            m = 0.0
        else:
            m = (n*sumxy - sumx*sumy) / denom
        angle_rad = math.atan2(m, 1.0)
        return math.degrees(angle_rad)
    else:
        # vertical: fit x = m*y + b
        sumx = sum(p[0] for p in points)
        sumy = sum(p[1] for p in points)
        sumyy = sum(p[1]*p[1] for p in points)
        sumxy = sum(p[0]*p[1] for p in points)
        denom = (n*sumyy - sumy*sumy)
        if denom == 0:
            m = 0.0
        else:
            m = (n*sumxy - sumy*sumx) / denom
        angle_rad = math.atan2(1.0, m) if m != 0 else math.pi/2
        # Convert so that 90deg means perfectly vertical
        # angle relative to +x axis:
        angle_relative_to_x = math.atan2(height-1, m*(height-1)) if m != 0 else math.pi/2
        return math.degrees(angle_rad)

# --- Load or generate image ---
input_path = Path(input_image_path) if input_image_path else None
loaded = read_pnm_ascii(input_path) if input_path else None

if loaded is not None:
    if loaded['mode'] == 'L':
        img_gray = loaded['data']
        height = loaded['height']; width = loaded['width']
    else:
        # convert RGB to grayscale (luma)
        height = loaded['height']; width = loaded['width']
        img_gray = [[0 for _ in range(width)] for _ in range(height)]
        for y in range(height):
            for x in range(width):
                r, g, b = loaded['data'][y][x]
                v = int(0.299*r + 0.587*g + 0.114*b)
                img_gray[y][x] = v
    input_used = input_path
else:
    img_gray = generate_sample(width, height, seam_direction)
    input_used = None

# --- Compute edges and seam ---
edges = sobel_edges(img_gray)
seam_points = estimate_seam_points(img_gray, seam_direction)
angle_deg = estimate_line_angle_deg(seam_points, seam_direction)

# --- Save outputs ---
input_out = output_dir / ('input.pgm' if input_used is None else input_path.name)
edges_out = output_dir / 'edges.pgm'
overlay_out = output_dir / 'seam_overlay.ppm'
points_out = output_dir / 'seam_points.csv'
report_out = output_dir / 'geometry_and_results.json'

if input_used is None:
    save_pgm_ascii(input_out, img_gray)
else:
    # If original was PPM and we converted to gray, still write grayscale copy
    save_pgm_ascii(output_dir / 'input_gray.pgm', img_gray)

save_pgm_ascii(edges_out, edges)

# Build overlay (RGB) from grayscale + seam in red
h = len(img_gray); w = len(img_gray[0]) if h > 0 else 0
overlay = [[[img_gray[y][x]]*3 for x in range(w)] for y in range(h)]
for x_or_y, y_or_x in seam_points:
    if seam_direction == 'horizontal':
        x = x_or_y; y = y_or_x
    else:
        x = x_or_y; y = y_or_x
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            yy = y + dy; xx = x + dx
            if 0 <= yy < h and 0 <= xx < w:
                overlay[yy][xx] = [255, 0, 0]

save_ppm_ascii(overlay_out, overlay)

# Save CSV of seam points
with points_out.open('w') as f:
    if seam_direction == 'horizontal':
        f.write('x,y\n')
    else:
        f.write('x,y\n')
    for p in seam_points:
        f.write(f'{p[0]},{p[1]}\n')

# Save JSON report
report = {
    'joint': {
        'type': joint_type,
        'root_opening_mm': root_opening_mm,
        'bevel_angle_deg': bevel_angle_deg,
        'land_thickness_mm': land_thickness_mm,
        'torch_angle_deg': torch_angle_deg,
    },
    'image': {
        'width': w,
        'height': h,
        'input_image': str(input_out.name),
    },
    'seam': {
        'estimated_angle_deg': angle_deg,
        'num_points': len(seam_points),
        'direction': seam_direction,
        'points_csv': str(points_out.name),
        'overlay_image': str(overlay_out.name),
    }
}
with report_out.open('w') as f:
    json.dump(report, f, indent=2)

print(f"Wrote outputs to: {output_dir}")
print(f"- Input: {input_out if input_used is None else (output_dir / 'input_gray.pgm')} (PGM)")
print(f"- Edges: {edges_out} (PGM)")
print(f"- Overlay: {overlay_out} (PPM)")
print(f"- Seam points: {points_out} (CSV)")
print(f"- Report: {report_out} (JSON)")
PY

# Print a brief summary
cat <<DONE
Joint geometry:
  type=$JOINT_TYPE, root-opening=$ROOT_OPENING mm, bevel-angle=$BEVEL_ANGLE deg, land=$LAND_THICKNESS mm, torch-angle=$TORCH_ANGLE deg
Outputs: $OUTPUT_DIR_ABS
DONE
