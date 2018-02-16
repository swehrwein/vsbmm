#!/usr/bin/env bash
[ $# -ne 1 ] && { echo "Usage $0 dataset"; exit 1; }

DS=$1
TRACKDIR=${DAVIS_ROOT}/JPEGImages/480p/${DS}/imagesResults
DATFILES=( ${TRACKDIR}/imagesTracks*.dat )
DATFILE=${DATFILES[0]}
TTFILE="${TRACKDIR}/tracks.tt"

set -x
${BGSEG}/trackseg/build/trackadapter brox tt $DATFILE $TTFILE -t=ransac
