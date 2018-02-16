[ $# -lt 2 ] && { echo "Usage $0 dataset tag"; exit 1; }

DS=$1
TAG=$2

OUTDIR=${DAVIS_RESULTS}/${TAG}

set -x
python ${BVS}/scripts/seg_compare.py ${DAVIS_INPUT}/${DS}/%05d.jpg ${OUTDIR}/${DS}/compare%06d.png ${OUTDIR}/${DS}/%06d.png ${DAVIS_GT}/${DS}/%05d.png 

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/compare%06d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${TAG}_compare.mp4
