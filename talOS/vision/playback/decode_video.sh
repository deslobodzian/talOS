#!/usr/bin/env bash
# Decodes a video file into a recording the camera node and recording_eval
# play: lossless 8-bit grayscale PNGs plus timestamps.csv carrying each
# frame's presentation time from the container (ffprobe best_effort_timestamp),
# so variable frame rates keep their own timeline.
#
#   talOS/vision/playback/decode_video.sh VIDEO OUT_DIR [ffmpeg -vf filter]
#
# The repo's OpenCV videoio has no FFmpeg backend on Linux, so this uses the
# system ffmpeg/ffprobe. Treat the video as untrusted data: only ffmpeg and
# ffprobe read it. OUT_DIR must not exist yet.
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "usage: $0 VIDEO OUT_DIR [ffmpeg -vf filter, e.g. scale=1280:-2]" >&2
  exit 2
fi
video=$1
out=$2
filter=${3:-}
command -v ffmpeg >/dev/null && command -v ffprobe >/dev/null || {
  echo "$0: ffmpeg and ffprobe are required" >&2
  exit 1
}
if [[ -e $out ]]; then
  echo "$0: $out exists; give a new directory" >&2
  exit 1
fi
mkdir -p "$out"

vf=format=gray
[[ -n $filter ]] && vf="$filter,format=gray"
# -fps_mode passthrough: one PNG per decoded frame, none duplicated or dropped,
# so PNG n is ffprobe's frame n.
ffmpeg -nostdin -loglevel error -i "$video" -map 0:v:0 -an -sn -dn \
  -fps_mode passthrough -vf "$vf" "$out/%06d.png"

# Presentation time of every decoded frame of the first video stream, seconds.
mapfile -t times < <(ffprobe -v error -select_streams v:0 \
  -show_entries frame=best_effort_timestamp_time -of csv=p=0 "$video")
count=$(find "$out" -maxdepth 1 -name '*.png' | wc -l)
if [[ ${#times[@]} -ne $count ]]; then
  echo "$0: ffprobe found ${#times[@]} frames, ffmpeg wrote $count" >&2
  exit 1
fi
{
  echo "file,pts_ns"
  for i in "${!times[@]}"; do
    # Seconds with up to 6 decimals, to integer nanoseconds without floats.
    t=${times[$i]%,}
    if [[ ! $t =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
      echo "$0: frame $((i + 1)) has no usable timestamp ($t)" >&2
      exit 1
    fi
    whole=${t%%.*}
    frac=${t#*.}
    [[ $t == *.* ]] || frac=0
    frac=$(printf '%-9s' "$frac" | tr ' ' 0)
    printf '%06d.png,%d\n' $((i + 1)) $((10#$whole * 1000000000 + 10#${frac:0:9}))
  done
} >"$out/timestamps.csv"
echo "$out: $count frames"
