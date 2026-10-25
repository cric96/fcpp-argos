#!/bin/bash
# Records an experiment with ARGoS' headless frame grabbing (needs a display) into docs/:
#   scripts/record.sh <experiment> <seconds> gif [speedup]   animation, default 3x real time
#   scripts/record.sh <experiment> <seconds> png             last frame only
set -euo pipefail
cd "$(dirname "$0")/.."
name=$1 secs=$2 kind=$3 speedup=${4:-3}
frames=$(mktemp -d)
trap 'rm -r "$frames"' EXIT

sed -e "s/length=\"0\"/length=\"$secs\"/" \
    -e "s|<qt-opengl>|<qt-opengl autoplay=\"true\"><frame_grabbing directory=\"$frames\" headless_grabbing=\"true\" headless_frame_size=\"800x800\" headless_frame_rate=\"$speedup\" />|" \
    "experiments/$name.argos" > "build/$name.rec.argos"
argos3 -c "build/$name.rec.argos" > "build/$name.rec.log" 2>&1

mkdir -p docs
crop="crop=ih*0.92:ih*0.92" # square around the arena
if [ "$kind" = png ]; then
    ffmpeg -loglevel error -y -i "$(ls "$frames"/*.png | tail -1)" -vf "$crop,scale=640:-1:flags=lanczos" "docs/$name.png"
else
    # frames are grabbed every <speedup> ticks; 10 ticks = 1 s
    ffmpeg -loglevel error -y -framerate 10 -pattern_type glob -i "$frames/*.png" \
        -vf "$crop,scale=420:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=none" \
        "docs/$name.gif"
fi
ls -lh "docs/$name.$kind"
