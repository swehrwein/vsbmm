
[ $# -lt 2 ] && { echo "Usage $0 dataset tag"; exit 1; }

DS=$1
MODEL=homography
TAG=$2
set -x

$BGSEG/scripts/trackransac.sh $DS $MODEL $TAG "$@"
$BGSEG/scripts/bseg.sh $DS $MODEL $TAG -o $BGSEG/bvs/bestOpts.txt
