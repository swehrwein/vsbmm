This is an implementation of
    Scott Wehrwein and Richard Szeliski
    Video Segmentation with Background Motion Models
    BMVC 2017

If you use this in your research, please cite our paper.

============
Dependencies
============
Please report any dependencies not listed here using github issues.

You'll need:
    Imagemagick
    Python
    Eigen3
    CMake
    OpenCV
    Theia
    

This relies on some external libraries you'll need to download:
    1. Dense Point Tracking:
       N. Sundaram, T. Brox, K. Keutzer
       Dense point trajectories by GPU-accelerated large displacement optical flow,
       European Conference on Computer Vision (ECCV)
       Crete, Greece, Springer, LNCS, Sept. 2010.

       Download the executable from
        https://lmb.informatik.uni-freiburg.de/resources/software.php
       and place it in the tracking/densetrack directory. Use the makefile
       to build the tracker there so you have an executable at
       tracking/densetrack/tracking.

    2. Theia - download and install from http://theia-sfm.org/

    3. maxflow-v3.01
       Y. Boykov and V. Kolmogorov
       An experimental comparison of min-cut/max- flow algorithms for
       energy minimization in vision
       IEEE Transactions on Pattern Analysis and Machine Intelligence
       vol. 26, no. 9, pp. 1124-1137, Sept. 2004.

       Download the maxflow-v3.01 package from
        http://vision.csd.uwo.ca/code/
       and place the code in the bvs folder, i.e. you should have
        bvs/maxflow-v3.01/maxflow.cpp, etc.

    4. Google Sparsehash
       https://github.com/sparsehash/sparsehash
       Install using their instructions. If it's installed somewhere other than 
       /usr/local/include/sparsehash, change bvs/CMakeLists.txt accordingly.

=====
Setup
=====
The scripts depend on having the following evironment variables set:
    ${DAVIS_ROOT} - root of a given set of davis images, s.t.
        ${DAVIS_ROOT}/JPEGImages/480p/bear/?????.jpg contains the bear inputs

    ${BGSEG} - path to this directory

There are two components to build using CMake:
    1. trackseg
    2. bvs

For each component, you may need to tweak the CMakeLists.txt file to correctly locate
OpenCV (on Mac I had to manually set OpenCV_Dir). Then,
    $ cd trackseg
    $ mkdir build
    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    $ make
and likewise for bvs.

===================
Overview of scripts
===================

scripts/ contains scripts that run the method on DAVIS datasets:
    pipeline.sh - runs all steps, beginning with tracking. Note tracking only works on linux.
                  Each of the following is a step run by pipeline.
                  Example usage:
                    $ bash pipeline.sh bear my_results

                  where my_results is the desired output folder name for your results.
                  Segmentation results are placed in
                    $DAVIS_ROOT/Results/Segmentations/480p/my_results/bear

    brox2tt.sh - converts the Brox tracking dat file into a
                 .tt file to be read into a TrackTable.

    trackransac.sh - runs our method to classify tracks using background motion models
                     places results in $DAVIS_ROOT/Results/Dataterm/,

    bseg.sh - takes the track confidences from trackransac.sh and uses our variant of 
              bilateral video segmentation to produce a dense segmentation.

    segment.sh - once you've run tracking, you don't need to recompute tracks. this script
                 runs trackransac.sh and bseg.sh, using the already-computed tracking data.

