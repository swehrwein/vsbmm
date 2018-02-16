#!/usr/bin/env bash
[ $# -lt 2 ] && { echo "Usage $0 metrics method [outfile]"; exit 1; }

METRICS=$1
METHOD=$2

export PYTHONPATH=~/src/davis/python/lib
set -x
python ~/src/davis/python/experiments/eval_all.py --compute --techniques ${METHOD} --metrics ${METRICS}

if [ $# -lt 3 ]; then
    python ~/src/davis/python/tools/eval_view.py ~/src/davis/data/DAVIS/Results/Evaluation/480p/${METHOD}.h5
else
    python ~/src/davis/python/tools/eval_view.py ~/src/davis/data/DAVIS/Results/Evaluation/480p/${METHOD}.h5 > $3
fi


#[ $# -lt 3 ] && { echo "Usage $0 dataset model label [-p prefix]"; exit 1; }

#DS=$1
#MODEL=$2
#LABEL=$3
#shift
#shift
#shift

## defaults:
#PREFIX="bvs"

#while getopts "i:f:p:" opt; do
    #case "$opt" in
    #h|\?)
        #show_help
        #exit 0
        #;;
    #p)  PREFIX=$OPTARG
        #;;
    #esac
#done

##echo "${DAVIS_DIR}/${DS}/results/${PREFIX}_${MODEL:0:1}_"
#RESULTDIR=~/data/DAVIS/Results/Segmentations/480p/${LABEL}/${DS}/
#mkdir -p $RESULTDIR
#for f in ${DAVIS_DIR}/${DS}/results/${PREFIX}_${MODEL:0:1}_seg[0-9]*.png; do
    #cp $f $RESULTDIR`echo $f | egrep -o [0-9]{5}.png`
#done



