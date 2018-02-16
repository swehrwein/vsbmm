#!/usr/bin/env bash
[ $# -lt 1 ] && { echo "Usage $0 dataset"; exit 1; }

DS=$1
shift

# defaults:
NF=0
PREFIX="mybvs"
PROFILE="bvs"

while getopts "f:n" opt; do
    case "$opt" in
    h|\?)
        echo "Usage: bseg.sh ds model [-f nframes=0]"
        exit 0
        ;;
    f)  NF=$OPTARG
        ;;
    n)  PREFIX="${PREFIX}fast"
        PROFILE="${PROFILE}fast"
        ;;
    esac
done

OUTDIR=${DAVIS_RESULTS}/${PREFIX}
GTDIR=${DAVIS_GT}/${DS}/
mkdir -p ${OUTDIR}/${DS}

set -x

time ${BVS}/build/bseg ${DAVIS_INPUT}/${DS}/ 5 ${OUTDIR}/${DS}/ ${NF} --gt=${GTDIR} --profile=${PROFILE}

set +x

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/%05d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${PREFIX}.mp4

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/seg%05d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${PREFIX}masked.mp4
