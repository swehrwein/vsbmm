#!/usr/bin/env bash
USAGE="Usage: bseg.sh ds model dataterm [-f nframes=0] [-p prefix=bgseg] [-o optfile]"
[ $# -lt 2 ] && { echo $USAGE; exit 1; }

DS=$1
MODEL=$2
DATATERM=$3
shift
shift
shift

# defaults:
NF=0
PREFIX="bgseg"
PROFILE="--profile=bg"
TTFILE=${DAVIS_ROOT}/Results/Dataterm/${DATATERM}/${DS}/${MODEL}final.tt

while getopts "f:p:no:i" opt; do
    case "$opt" in
    h|\?)
        echo $USAGE
        exit 0
        ;;
    f)  NF=$OPTARG
        ;;
    p)  PREFIX=$OPTARG
        ;;
    n)  PROFILE="--profile=bgnn"
        ;;
    o)  PROFILE=''
        OPT="-optsfile=${OPTARG}"
        ;;
    i)  TTFILE=${DAVIS_ROOT}/Results/Dataterm/${DATATERM}/${DS}/${MODEL}init.tt
    esac
done

OUTDIR=${DAVIS_RESULTS}/${PREFIX}_${MODEL:0:1}
mkdir -p ${OUTDIR}/${DS}

set -x

time ${BVS}/build/bseg ${DAVIS_INPUT}/${DS}/ ${OUTDIR}/${DS}/ ${NF} -trackfile=${TTFILE} ${PROFILE} $OPT

set +x

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/%05d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${PREFIX}_${MODEL:0:1}.mp4

ffmpeg -hide_banner -loglevel panic -y -i ${OUTDIR}/${DS}/seg%05d.png -pix_fmt yuv420p -c:v libx264 -crf 18 ${OUTDIR}/${DS}_${PREFIX}_${MODEL:0:1}masked.mp4
