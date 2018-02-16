#include "Grid.h"

#include "maxflow-v3.01/graph.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <../common/TrackTable.h>
#include <../common/TrackTable-inl.h>
#include <../common/RansacData.h>

#define WOLF(x) std::cout << "wolf " << x << std::endl;

//#define DIAGNOSTICS
#undef DIAGNOSTICS

using namespace cv;
using namespace std;

typedef Graph<float, float, double> GraphCut;

#define ND 6
typedef Vec4f Val;
typedef Grid<Val, ND> GridT;
typedef GridT::Ind Ind;
typedef GridT::Indf Indf;

#include "BvsOpts.h"
BvsOpts profile;

string makeFn(const string outFile, const string label, const int digits,
               const int i) {
  ostringstream ofn;
  ofn << outFile << label << setfill('0') << setw(digits) << i << ".png";
  return ofn.str();
}


void byte2float(const Mat1b& img, Mat1f& result) {
  img.convertTo(result, CV_32FC1, 1.0/255);
}

void byte2float(const Mat3b& img, Mat3f& result) {
  img.convertTo(result, CV_32FC3, 1.0/255);
}

void blendBuffer(Mat3b& frame, Mat4b overlayBGRA, bool flipAxes=false) {
  assert(frame.rows == overlayBGRA.rows && frame.cols == overlayBGRA.cols);
  for (int r = 0; r < frame.rows; ++r) {
    Vec3b* frameRowPtr = frame.ptr<Vec3b>(r);
    Vec4b* overlayRowPtr = overlayBGRA.ptr<Vec4b>(r);
    for (int c = 0; c < frame.cols; ++c) {
      float alpha = 1.0 / 255 * overlayRowPtr[c](3);
      Vec4f oVal(overlayRowPtr[c]);
      Vec3f oColor(oVal(0), oVal(1), oVal(2));
      frameRowPtr[c] =
          Vec3b((1 - alpha) * Vec3f(frameRowPtr[c]) + alpha * oColor);
    }
  }
}

void loadFrames(const string inputDir, const int maxFrames,
                vector<Mat3b> &frames) {
  cout << "Loading images..." << endl;

  ifstream bmffile(inputDir + "/images.bmf");
  int nFrames;
  bmffile >> nFrames;
  if (maxFrames > 0 && nFrames > maxFrames) {
    nFrames = maxFrames;
  }
  string tmp;
  getline(bmffile, tmp);
  for (int i = 0; i < nFrames; ++i) {
    string fn;
    getline(bmffile, fn);
    Mat3b im = imread(inputDir + "/" + fn);
    frames.push_back(im);
  }
  cout << "Loaded " << frames.size() << " frames." << endl;
}

void loadVideoFrames(const string inputFn, const int nFrames,
                     vector<Mat3b> &frames) {
  cout << "Loading video..." << flush;
  VideoCapture cap(inputFn);
  if (!cap.isOpened()) {
    cout << "Could not open video " << inputFn << endl;
    return;
  }
  Mat3b frame;
  // throw out a few frames from the beginning
  cap.read(frame);
  cap.read(frame);

  frames.resize(nFrames);
  bool readSuccess = true;
  for (int f = 0; f < nFrames; ++f) {
    if (!cap.read(frames[f])) {
      frames.resize(f);
      cout << "ran out of frames at frame " << f << endl;
      break;
    }
  }
  cout << "done." << endl;
}

void splatTrackSeg(const string ttFileName, const vector<Mat3b> inFrames,
                   GridT &g) {
  // load tracks from file
  TrackTable<Observation2d, RansacData> tracks;
  FILE* infile = fopen(ttFileName.c_str(), "r");
  tracks.deserialize(infile);
  fclose(infile);

  int NR = inFrames[0].rows;
  int NC = inFrames[1].cols;
  int NF = inFrames.size();

  vector<Mat1b> noTracks;
  if (profile.noTracksRadius > 0) {
    noTracks.resize(NF);
    for (int f = 0; f < NF; ++f) {
      noTracks[f].create(NR, NC);
      noTracks[f].setTo(0);
      for (int r = 0; r < NR; r += profile.noTracksSpacing) {
        unsigned char* rPtr = noTracks[f].ptr<unsigned char>(r);
        for (int c = 0; c < NC; c += profile.noTracksSpacing) {
          rPtr[c] = 1;
        }
      }
    }
  }



  for (int tid = 0; tid < tracks.numTracks(); ++tid) {
    if (!tracks.hasTrack(tid)) {
      continue;
    }

    Vec2f offset(((float)inFrames[0].cols) / 2, ((float)inFrames[0].rows) / 2);
    Track<Observation2d, RansacData>& t = tracks.track(tid);
    if (t.length() <= 5) {
      continue;
    }

    // value is the same across all observations
    float inlierWt = t.data.inlWts[0];
    Val val(0.0, 0.0, 0.0, 0.0);
    val(inlierWt < 0.5 ? 1 : 0) = 2 * std::abs(inlierWt - 0.5);

    // weight val by track length
    //float lengthScale =
        //std::max(0.2, std::min(1.0, (double)t.length() / inFrames.size()));
    //val = val * lengthScale;

    // splat the value to each of the track's coordinates
    int lastFrame = std::min((size_t)t.lastFrame()+1, inFrames.size());
    for (int f = t.firstFrame(); f < lastFrame; ++f) {
      Vec2f loc = t.obs(f).loc + offset;
      float r = loc(1);
      float c = loc(0);
      int rI = round(r);
      int cI = round(c);

      if (profile.noTracksRadius > 0) {
        // draw circle in noTracks[f] at rI, cI with radius noTracksRadius
        // to black out any noTracks points near this track
        circle(noTracks[f], Point2i(cI, rI), profile.noTracksRadius, Scalar(0),
               -1);
      }

      const Vec3b& color = inFrames[f].at<Vec3b>(rI,cI);

      Indf index(f, r, c, color(0), color(1), color(2));
      g.splat(index, val);
    }
  }

  // splat noTracks data term points
  if (profile.noTracksRadius > 0) {
    Val val(profile.noTracksCost, 0.0, 0.0, 0.0);
    for (int f = 0; f < NF; ++f) {
      //imshow("nt", noTracks[f] * 255); waitKey(0);
      for (int r = 0; r < NR; r += profile.noTracksSpacing) {
        const unsigned char* rPtr = noTracks[f].ptr<unsigned char>(r);
        for (int c = 0; c < NC; c += profile.noTracksSpacing) {
          if (rPtr[c] == 1) {
            const Vec3b& color = inFrames[f].at<Vec3b>(r,c);
            Indf index(f, r, c, color(0), color(1), color(2));
            g.splat(index, val);
          }
        }
      }
    }
  }
}

void splatFrame(GridT& g, const Mat3b input, const int f) {
  for (int r = 0; r < input.rows; ++r) {
    const Vec3b* inRowPtr = input.ptr<Vec3b>(r);
    for (int c = 0; c < input.cols; ++c) {
      Indf index(f, r, c, inRowPtr[c](0), inRowPtr[c](1), inRowPtr[c](2));
      Val value(0.0, 0.0, 1.0, 0.0);
      g.splat(index, value);
    }
  }
}

void splatDataTermFrame(GridT &g, const Mat2f dataTerm, const Mat3b inFrame,
                        const int f) {
  for (int r = 0; r < inFrame.rows; ++r) {
    const Vec3b* inRowPtr = inFrame.ptr<Vec3b>(r);
    const Vec2f* dtRowPtr = dataTerm.ptr<Vec2f>(r);
    for (int c = 0; c < inFrame.cols; ++c) {
      Indf index(f, r, c, inRowPtr[c](0), inRowPtr[c](1), inRowPtr[c](2));
      Val value(dtRowPtr[c](0), dtRowPtr[c](1), 0.0, 0.0);
      g.splat(index, value);
    }
  }
}

// solve graph cut on the current grid; put resuling labels in val(3) of each cell
void graphCut(GridT &g) {
  // setup a mapping from grid indices to graph node indices
#if STLHASH
  std::unordered_map<Ind, int, SeqHash<Ind, ND>> nodeIndexMap(g.size());
#else
  google::dense_hash_map<Ind, int, SeqHash<Ind, ND>> nodeIndexMap(g.size());
  Ind emptyKey;
  for (int i = 0; i < ND; ++i) {
    emptyKey[i] = -1;
  }
  nodeIndexMap.set_empty_key(emptyKey);
#endif
  int ind = 0;
  for (auto it = g.begin(); it != g.end(); ++it) {
    nodeIndexMap[(*it).first] = ind++;
  }

#ifdef DIAGNOSTICS
  vector<int> unaries, smooths;
  double unaryTotal = 0.0, smoothTotal = 0.0;
#endif

  cout << "Setup graph..." << endl;
  GraphCut gc(g.size(), g.size()*pow(2,ND));
  gc.add_node(g.size());
  for (typename GridT::MapType::const_iterator it = g.begin(); it != g.end();
       ++it) {
    // for (auto it = g.begin(); it != g.end(); ++it) {
    Ind ind = (*it).first;
    Val val = (*it).second;

    //cout << "graph " << ind << " " << val << endl;
    //cout << val(0) << " " << val(1) << endl;

    // set unary costs
    float bg_cost = profile.unaryWeight * val(1);
    float fg_cost = profile.unaryWeight * val(0);

#ifdef DIAGNOSTICS
    unaries.push_back(fg_cost);
    unaries.push_back(bg_cost);
    unaryTotal += fg_cost + bg_cost;
#endif
    gc.add_tweights(nodeIndexMap[ind], bg_cost, fg_cost);

    // set edge costs wrt all neighbors
    for (int d = 0; d < ND; ++d) {
      for (int pos = 0; pos < 2; ++pos) {
        Ind nbr = ind;
        nbr[d] += (pos) ? 1 : -1;
        if (g.has(nbr)) {
          Val nbrVal = g[nbr];
          float indMass = val(2);
          float nbrMass = nbrVal(2);

          //Indf dist = nbr - ind;
          //multiply(dist, profile.dimWeights, dist);

          //float dTd = norm(dist, NORM_L2SQR);
          float cap = profile.smoothWeight * (indMass * nbrMass);
          cap *= exp(-0.5 * profile.dimWeights(d));
          cap = std::max(profile.smoothReg, cap);
          //float cap
          //float cap = (float)(wt * exp(-0.5 * dTd));

#ifdef DIAGNOSTICS
          smooths.push_back(cap);
          smoothTotal += cap;
#endif
          gc.add_edge(nodeIndexMap[ind], nodeIndexMap[nbr], cap, cap);
        }
      }
    }
  }

#ifdef DIAGNOSTICS
  // print some statistics about costs
  cout << "unary avg " << unaryTotal / unaries.size()
    << " max " << *(max_element(unaries.begin(), unaries.end())) << " "
    << " min " << *(min_element(unaries.begin(), unaries.end())) << endl;
  cout << "smooth avg " << smoothTotal / smooths.size()
    << " max " << *(max_element(smooths.begin(), smooths.end())) << " "
    << " min " << *(min_element(smooths.begin(), smooths.end())) << endl;
#endif

  // solve graph cut
  cout << "Solve graphcut with " << gc.get_node_num() << " nodes and ";
  cout << gc.get_arc_num() << " edges." << endl;
  double cost = gc.maxflow();
  cout << "cost " << cost << " ";

  // tally up the unary cost
  double unary_cost = 0;
  double inv_unary_cost = 0;
  for (auto it = g.begin(); it != g.end(); ++it) {
    bool fg = gc.what_segment(nodeIndexMap[(*it).first]) == GraphCut::SOURCE;
    // store result in val(3)
    (*it).second(3) = fg ? 1.0f : 0.0f;
    //cout << (*it).first << (*it).second << endl;
    unary_cost += profile.unaryWeight * (fg ? (*it).second(0) : (*it).second(1));
  }
  cout << "(" << unary_cost << " unary)" << endl;
}

void sliceFrame(GridT& g, const Mat3b& inFrame, Mat1b& outFrame, const int f) {
  for (int r = 0; r < inFrame.rows; ++r) {
    const Vec3b* inRowPtr = inFrame.ptr<Vec3b>(r);
    unsigned char* outRowPtr = outFrame.ptr<unsigned char>(r);
    for (int c = 0; c < inFrame.cols; ++c) {
      Indf index(f, r, c, inRowPtr[c](0), inRowPtr[c](1), inRowPtr[c](2));
      Val slicedVal = g.slice(index);
      outRowPtr[c] = slicedVal(3) * 255.0;
    }
  }
}

#ifdef DIAGNOSTICS
void sliceDataCosts(GridT &g, const vector<Mat3b> inFrames,
                    vector<Mat1b> &fgCosts, vector<Mat1b> &bgCosts) {
  double mn, mx;

  fgCosts.resize(inFrames.size());
  bgCosts.resize(inFrames.size());
  for (int f = 0; f < inFrames.size(); ++f) {
    fgCosts[f].create(inFrames[f].rows, inFrames[f].cols);
    bgCosts[f].create(inFrames[f].rows, inFrames[f].cols);

    Mat1f fgCost(inFrames[f].rows, inFrames[f].cols);
    Mat1f bgCost(inFrames[f].rows, inFrames[f].cols);

    for (int r = 0; r < inFrames[f].rows; ++r) {
      const Vec3b* inRowPtr = inFrames[f].ptr<Vec3b>(r);
      float* fgRowPtr = fgCost.ptr<float>(r);
      float* bgRowPtr = bgCost.ptr<float>(r);
      for (int c = 0; c < inFrames[f].cols; ++c) {
        Indf index(f, r, c, inRowPtr[c](0), inRowPtr[c](1), inRowPtr[c](2));
        Val slicedVal = g.slice(index);
        fgRowPtr[c] = slicedVal(0);
        bgRowPtr[c] = slicedVal(1);
      }
    }
    cout << f;
    // fg data cost
    pow(fgCost, 0.25, fgCost);
    minMaxLoc(fgCost, &mn, &mx);
    fgCost.convertTo(fgCosts[f], CV_8UC1, 255.0/mx);
    //cout << "fg max " << mx << " sum " << sum(fgCost) << endl;
    //cvtColor(disp, disp3b, CV_GRAY2BGR);
    //imshow("fg", disp); waitKey(0);

    cout << " maxfg: " << mx;
    // bg data cost
    pow(bgCost, 0.1, bgCost);
    minMaxLoc(bgCost, &mn, &mx);
    bgCost.convertTo(bgCosts[f], CV_8UC1, 255.0/mx);
    //cout << "bg max " << mx << " min " << mn << " sum " << sum(bgCost) << endl;

    cout << " maxbg: " << mx << endl;
  }
}
#endif

int main(int argc, char** argv) {

  cv::CommandLineParser parser(argc, argv,
    "{help h    | | print this message }"
    "{@inputdir | | input frame base dir}"
    "{@output   | | output frame base filename }"
    "{@nframes  |0| max number of frames (0 == all) }"
    "{trackfile | | tt file with track inlier weights}"
    "{gt        | | ground truth frame base filename}"
    "{profile   | | parameters profile (bgseg, bvs)}"
    "{optsfile  | | file specifying parameters}"
    "{vidin     | | input is a video}");

  if (parser.has("help") || argc == 1) {
    parser.printMessage();
    return 0;
  }
  if (!parser.check()) {
    parser.printErrors();
    return 0;
  }

  string optfile = parser.get<string>("optsfile");
  string prof = parser.get<string>("profile");
  if (prof != "") {
    getProfile(prof, profile);
  } else if (optfile != "") {
    readOptions(optfile, profile);
  } else {
    cout << "Must specify profile name or options file." << endl;
  }

  string inputDir = parser.get<String>("@inputdir");
  string outFile = parser.get<String>("@output");
  int maxFrames = parser.get<int>("@nframes");
  string trackfile = parser.get<string>("trackfile");

  string groundTruthBase = "";
  if (profile.gtFrames != 0) {
    groundTruthBase = parser.get<String>("gt");
  }
  bool vidIn = parser.has("vidin");
  //bool vidOut = parser.has("vidout");
  int digits = 6;

  // create stacks
  vector<Mat3b> inFrames;
  vector<Mat3b> gtFrames;

  // load input from video or images
  if (vidIn) {
    loadVideoFrames(inputDir, maxFrames, inFrames);
  } else {
    loadFrames(inputDir, maxFrames, inFrames);
  }

  // convert input frames to Luv
  for (int f = 0; f < inFrames.size(); ++f) {
    cvtColor(inFrames[f], inFrames[f], CV_BGR2Luv);
  }

  if (profile.gtFrames < 0) {
    profile.gtFrames = inFrames.size();
  }
  // load gt frames if applicable
  if (profile.gtFrames > 0) {
    loadFrames(groundTruthBase, profile.gtFrames, gtFrames);
  }

  // setup grid dimensions: (frame, row, col, L, u, v)
  Indf inShapes(inFrames.size(), inFrames[0].rows, inFrames[0].cols, 255.0,
                255.0, 255.0);
  Ind shape;

  Indf& scales = profile.dimScales;
  for (int d = 0; d < ND; ++d) {
    // if dimScales[d] was negative, use value from gridSizes instead
    if (scales[d] < 0) {
      // set scale so shape will multiply out to max index = gridSizes[d] - 1
      scales[d] = ((float)profile.gridSizes[d]-1) / inShapes[d];
    }
    scales[d]; // dodge integer issues
    // scale and ceil it to get the final grid shape
    shape[d] = std::ceil(scales[d] * inShapes[d]);
  }

  cout << "Grid shape: " << shape << endl;
  cout << "Dim scales: " << scales << endl;

  GridT g(shape, scales, profile.interp);

  //// splat track inlier weights
  if (trackfile != "") {
    cout << "Splat tracks..." << endl;
    splatTrackSeg(trackfile, inFrames, g);
  }

  //// splat input video pixels
  cout << "Splat pixels..." << endl;
  for (int f = 0; f < inFrames.size(); ++f) {
    cout << "\r" << f << flush;
    splatFrame(g, inFrames[f], f);
  }

  //// splat gt data term
  cout << profile.gtFrames << endl;
  if (profile.gtFrames > 0) {
    cout << "Splat " << profile.gtFrames << " frames of groundtruth." << endl;
    for (int f = 0; f < gtFrames.size(); ++f) {
      cout << "\r" << f << flush;
      Mat1b gtC1;
      Mat1f gtFloat[2];
      Mat2f dataTermFrame;
      cvtColor(gtFrames[f], gtC1, CV_BGR2GRAY);
      byte2float(gtC1, gtFloat[1]);
      gtFloat[0] = 1.0 - gtFloat[1];

      merge(gtFloat, 2, dataTermFrame);
      splatDataTermFrame(g, dataTermFrame, inFrames[f], f);

    }
  }

  //// splat frame difference data term
  if (profile.dtDataTerm) {
    cout << "Splat frame difference data term..." << endl;
    for (int f = 0; f < inFrames.size() - 1; ++f) {
      cout << "\r" << f << flush;
      Mat3f frame1, frame2;
      byte2float(inFrames[f+1], frame2);
      byte2float(inFrames[f], frame1);

      //imgMag(frame2 - frame1, diffMag);
      // compute difference magnitude
      Mat3f sqr;
      pow(frame2 - frame1, 2, sqr);
      Mat1f diffMag;
      transform(sqr, diffMag, cv::Matx13f(1,1,1));
      pow(diffMag, 0.5, diffMag);

      //double mn, mx;
      //minMaxLoc(diffMag, &mn, &mx);
      //cout << "diffmag " <<  mn << " " << mx << endl;
      Mat1f x[2] = {0.01*Mat1f::ones(diffMag.rows, diffMag.cols), diffMag};
      Mat2f dataTerm;
      merge(x, 2, dataTerm);
      splatDataTermFrame(g, dataTerm, inFrames[f], f);
    }
  }

#ifdef DIAGNOSTICS
  int gridCells = 1;
  for (int d = 0; d < ND; ++d) { gridCells *= g.shape_[d]; }
  cout << "Grid has " << g.size() << " / " << gridCells << " ("
       << 100 * (float)g.size() / gridCells << "%) cells occupied." << endl;
#endif

  //g.blur();
  //profile.smoothWeight = 0.0;

  graphCut(g);

  // slice
  vector<Mat1b> slicedFrames(inFrames.size());
  vector<Mat1b> masks(inFrames.size());

  cout << "Slice..." << endl;
  for (int f = 0; f < slicedFrames.size(); ++f) {
    cout << "\r" << f << flush;
    slicedFrames[f].create(inFrames[0].rows, inFrames[0].cols);
    sliceFrame(g, inFrames[f], slicedFrames[f], f);

    medianBlur(slicedFrames[f], slicedFrames[f], 3);
    threshold(slicedFrames[f], masks[f], profile.segThresh, 1, THRESH_BINARY);
  }

  cout << endl << "Visualizations..." << endl;
  // save out masks and visualizations
  for (int f = 0; f < masks.size(); ++f) {
    Mat3b mask3b, out;
    cvtColor(masks[f], mask3b, CV_GRAY2BGR);
    multiply(mask3b, inFrames[f], out);
    cvtColor(out, out, CV_Luv2BGR);

    string maskedFn = makeFn(outFile, "seg", digits, f);
    imwrite(maskedFn.c_str(), out);

    // save the mask
    string segFn = makeFn(outFile, "", digits, f);
    imwrite(segFn.c_str(), mask3b*255);

    // save a red-masked version
    Mat1b zero = Mat1b::zeros(mask3b.rows, mask3b.cols);
    Mat1b alph;
    alph = masks[f] * 96;
    Mat1b ch[] = {zero, zero, masks[f]*255, alph};
    Mat4b redblend;
    merge(ch, 4, redblend);
    Mat3b redFrame;
    cvtColor(inFrames[f], redFrame, CV_Luv2BGR);
    Mat1b redFrame1b;
    cvtColor(redFrame, redFrame1b, CV_BGR2GRAY);
    cvtColor(redFrame1b, redFrame, CV_GRAY2BGR);
    blendBuffer(redFrame, redblend);
    string redFn = makeFn(outFile, "red", digits, f);
    imwrite(redFn.c_str(), redFrame);

    // save non-thresholded sliced result
    string sliceFn = makeFn(outFile, "slice", digits, f);
    imwrite(sliceFn.c_str(), slicedFrames[f]);


  }
  cout << endl;

#ifdef DIAGNOSTICS
  vector<Mat1b> fgCosts;
  vector<Mat1b> bgCosts;
  sliceDataCosts(g, inFrames, fgCosts, bgCosts);
  for (int f = 0; f < inFrames.size(); ++f) {
    string fgCostFn = makeFn(outFile, "fgc", digits, f);
    imwrite(fgCostFn.c_str(), fgCosts[f]);

    string bgCostFn = makeFn(outFile, "bgc", digits, f);
    imwrite(bgCostFn.c_str(), bgCosts[f]);
  }
#endif
  return 0;
}


