#!/usr/bin/env bash
[ $# -lt 2 ] && { echo "Usage: gtseg.sh ds prefix [-f nframes=0] [-p profile | -o optfile]"; exit 1; }

DS=$1
PREFIX=$2
shift
shift

# defaults:
NF=0

while getopts "f:n" opt; do
    case "$opt" in
    h|\?)
        echo "argument error"
        exit 1
        ;;
    f)  NF=$OPTARG
        ;;
    p)  PROFILE="--profile=${OPTARG}"
        ;;
    o)  OPTFILE="--optsfile=${OPTARG}"
        ;;
    esac
done

OUTDIR=${DAVIS_RESULTS}/${PREFIX}
GTDIR=${DAVIS_GT}/${DS}/
mkdir -p ${OUTDIR}/${DS}

set -x

time ${BVS}/build/bseg ${DAVIS_INPUT}/${DS}/ 5 ${OUTDIR}/${DS}/ ${NF} --gt=${GTDIR} ${PROFILE} ${OPTFILE}

set +x

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/%05d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${PREFIX}.mp4

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/seg%05d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${PREFIX}masked.mp4
