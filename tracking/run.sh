#!/usr/bin/env bash
[ $# -ne 1 ] && { echo "Usage $0 dataset"; exit 1; }

# assumes DAVIS_ROOT points to davis data

DS_DIR=${DAVIS_ROOT}/JPEGImages/480p/$1

set -x
mogrify -format ppm "${DS_DIR}/*.jpg"

BMFFILE=${DS_DIR}/images.bmf
python $BGSEG/tracking/make_bmf.py ${DS_DIR} ${BMFFILE}

nframes=$(head -n1 $BMFFILE | awk '{print $1;}')
echo $nframes

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BGSEG}/tracking/densetrack ${BGSEG}/tracking/densetrack/tracking $BMFFILE 0 $nframes 8
