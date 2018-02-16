import os.path as osp
import numpy as np
import pipi

def seg_compare(infn, outfn, seg1fn, seg2fn, seg3fn=None):
    #vr1 = pipi.video.VideoReader(seg1fn)
    #vr2 = pipi.video.VideoReader(seg2fn)
    #h, w = vr.height, vr.width
    #vw = pipi.video.VideoWriter(outfn, h, w, pix_fmt="yuv420p")

    for i in xrange(1000):
        if not (osp.exists(seg1fn % i) and osp.exists(seg2fn % i)):
            break
        channels = [pipi.rgb2gray(pipi.imread(seg1fn % i)),
                    pipi.rgb2gray(pipi.imread(seg2fn % i))]
        if seg3fn != None:
            channels.append(pipi.rgb2gray(pipi.imread(seg3fn % i)))
        else:
            channels.append(np.zeros_like(channels[-1]))

        #channels = (frame1[:,:,np.newaxis], frame2[:,:,np.newaxis], frame2[:,:,np.newaxis])

        inframe = pipi.rgb2gray(pipi.imread(infn % i))
        overlay = np.concatenate([c[...,np.newaxis] for c in channels], axis=2);
        outframe = 0.4 * pipi.gray2rgb(inframe) + 0.6 * overlay
        pipi.imwrite(outframe, outfn % i)


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 4:
        print("usage: %s input output, seg1 seg2 [seg3]" % (sys.argv[0],))
        exit(0)
    seg_compare(*sys.argv[1:])
