#!/usr/bin/env bash

# dataset: name of a DAVIS sequence
# tag: unique tag for the experiment

# example:
#   $ bash pipeline.sh bear bgseg
#
# segmented frames are in $DAVIS_ROOT/Results/Segmentations/480p/bgseg/bear

[ $# -lt 2 ] && { echo "Usage $0 dataset tag"; exit 1; }

DS=$1
MODEL=homography
TAG=$2
shift
shift
shift

set -x

# run tracking
time bash $BGSEG/tracking/run.sh ${DS}

# convert brox's track output file to a serialized TrackTable
time $BGSEG/scripts/brox2tt.sh $DS

# run motion estimation and track segmentation
time $BGSEG/scripts/trackransac.sh $DS $MODEL $TAG

# do bilateral densification
time $BGSEG/scripts/bseg.sh $DS $MODEL $TAG -o $BGSEG/bvs/bestOpts.txt

