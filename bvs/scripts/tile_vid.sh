DS=$1

TPATT=${DAVIS_RESULTS}/submit_h/${DS}/tile%05d.png

BR=300
BR=$(($BR*8))
# one-pass:
ffmpeg -y -f image2 -i $TPATT -an -c:v libx264 -preset veryslow -crf 19 -pix_fmt yuv420p ${DAVIS_RESULTS}/submit_h/${DS}_tile.mp4

# two-pass:
#ffmpeg -y -f image2 -i $TPATT -r 30 -c:v libx264 -passlogfile ${DS} -preset veryslow -b:v ${BR}k -an -pix_fmt yuv420p -pass 1 -f mp4 /dev/null && \
#ffmpeg -y -f image2 -i $TPATT -r 30 -c:v libx264 -passlogfile ${DS} -preset veryslow -b:v ${BR}k -an -pix_fmt yuv420p -pass 2 -f mp4 ${DAVIS_RESULTS}/submit_h/${DS}_tile.mp4
