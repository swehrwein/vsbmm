
DS=$1

NFRAMES=$(head -n 1 $DAVIS_INPUT/${DS}/images.bmf | awk '{print $1}')

INPUT="${DAVIS_INPUT}/${DS}/%05d.jpg"
TRACK="${DAVIS_ROOT}/Results/Dataterm/maxerr_ud/${DS}/h_final_%06d.png"
REDSTR="${DAVIS_RESULTS}/submit_h/${DS}/red%06d.png"
#COMPARE="${DAVIS_RESULTS}/submit_h/${DS}/compare%06d.png"

TPATT=${DAVIS_RESULTS}/submit_h/${DS}/tile%05d.png


for i in `seq 0 $(($NFRAMES-1))`; do

    INP=$(printf $INPUT $i)
    if [[ $i -eq 0 ]]; then
        TRK=$INP
    else
        TRK=$(printf $TRACK $i)
    fi
    RED=$(printf $REDSTR $i)
    #CMP=$(printf $COMPARE $i)
    OUT=$(printf $TPATT $i)

    montage $TRK $RED -tile 2x1 -geometry 427x240+0+0 $OUT
done


