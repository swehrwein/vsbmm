import os
import sys

imgdir = sys.argv[1]

i = 0
files = []
while os.path.exists(os.path.join(imgdir, '%05d.ppm' % i)):
    files.append("%05d.ppm" % i)
    i += 1

with open(sys.argv[2], 'w') as outfile:
    outfile.write("%d %d\n" % (len(files), 1))
    for f in files:
        outfile.write(f + "\n")

