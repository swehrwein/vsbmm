#!/usr/bin/env bash
[ $# -lt 3 ] && { echo "Usage $0 dataset model tag"; exit 1; }

DS=$1
MODEL=$2
TAG=$3
shift
shift
shift

INPUTS=${DAVIS_ROOT}/JPEGImages/480p

TTFILE=${INPUTS}/$DS/imagesResults/tracks.tt

while getopts "bp:" opt; do
    case "$opt" in
    p)  PREFIX=$OPTARG
        ;;
    n)  PROFILE=bgnn
        ;;
    esac
done

INPUT="${INPUTS}/${DS}"

RESULTSDIR="${DAVIS_ROOT}/Results/Dataterm/${TAG}/${DS}"
mkdir -p ${RESULTSDIR}

VISFMT="${RESULTSDIR}/${MODEL:0:1}_%s_%06d.png"

TRACKOUT="${RESULTSDIR}/${MODEL}"

set -x
if  ! ${BGSEG}/trackseg/build/trackransac -m=$MODEL -v=$VISFMT -t=$TRACKOUT $TTFILE $INPUT; then
	echo "${DS} failed"
fi


