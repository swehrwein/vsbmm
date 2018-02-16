#include "TrackRansac.h"

#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <FeatureLib.h>
#include "Homography.h"
//#include "Rigid.h"
//#include "Rotation3d.h"
#include "TrackVis.h"
#include "TransformEstimator.h"
#include "TransformEstimator-inl.h"
//#include "Translation.h"

using namespace cv;
using namespace std;

#define WOLF(x) std::cout << "wolf " << x << std::endl;
//// implementation of TrackRansac functions

Vec3f homog(const Vec2f x) {
  return Vec3f(x(0), x(1), 1.0f);
}

Vec2f dehomog(const Vec3f x) {
  return Vec2f(x(0)/x(2), x(1)/x(2));
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::preprocessTracks() {
  // translate tracks so (0,0) is the optical axis and collect them in a vector
  offset_ = Vec2f(((float)width_) / 2, ((float)height_) / 2);
  inputTracks_.clear();
  for (int i = 0; i < tracks_.numTracks(); ++i) {
    if (tracks_.hasTrack(i)) {
      PTrack pt = tracks_.trackPtr(i);
      for (int j = pt->firstFrame(); j <= pt->lastFrame(); ++j) {
        pt->obs(j).loc -= offset_;
      }
      // exclude tracks that are too short
      if (pt->length() >= kMinOverlap) {
        inputTracks_.push_back(pt);
        pt->data.inlWts.resize(kNumModels); // initialize inlier weight to 0
      }
    }
  } // tracks
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::estimateModel(const int modelNum) {

  EstT estimator;

  vector<PTrack> outlierTracks;
  getOutlierTracks(modelNum, outlierTracks);

  cout << "Fit model " << modelNum
       << " with " << outlierTracks.size() << " tracks." << endl;
  bestModel_.resize(bestModel_.size() + 1);
  if (kInitMethod == TRACKWISE) {
    initModelTrackwise(outlierTracks, estimator);
  } else if (kInitMethod == FRAMEWISE) {
    initModelFramewise(outlierTracks, estimator);
  } else {
    cout << "invalid InitMethod value." << endl;
  }
  if (stackOutFn_ != "") {
    writeStackFile("init");
  }

  // output results of initialization
  for (int tI = 0; tI < inputTracks_.size(); ++tI) {
    float err = estimator.Error(inputTracks_[tI], bestModel_[modelNum]);
    float inlWt = robustWeight(err, kIrls_t2);
    if (err < kRansacErrorThresh) {
      inputTracks_[tI]->data.inlWts[modelNum] = 1;
    } else {
      inputTracks_[tI]->data.inlWts[modelNum] = 0;
    }
    //inputTracks_[tI]->data.inlWts[modelNum] = inlWt;
  }
  this->visPerFrame_ = true;
  writeOverlayFrames("init");
  this->visPerFrame_ = false;
  if (trackOutFn_ != "") {
    writeTrackFile("init");
  }

  for (int tI = 0; tI < inputTracks_.size(); ++tI) {
    float err = estimator.Error(inputTracks_[tI], bestModel_[modelNum]);
    float inlWt = robustWeight(err, kIrls_t2);
    inputTracks_[tI]->data.inlWts[modelNum] = inlWt;
  }


  // collect weights for IRLS
  vector<float> weights(outlierTracks.size());
  for (int tI = 0; tI < outlierTracks.size(); ++tI) {
    weights[tI] = outlierTracks[tI]->data.inlWts[modelNum];
  }

  // run IRLS
  estimateIrls(outlierTracks, weights, estimator, 50, kIrls_t2);

  // fill in inlier weights for all tracks
  for (auto&& t : inputTracks_) {
    float err = estimator.Error(t, bestModel_[modelNum]);
    t->data.inlWts[modelNum] = robustWeight(err, kIrls_t2);

    // fill in per-frame error
    OffsetVector<double> perFrame;
    bestModel_[modelNum]->getResiduals(t, perFrame);
    t->data.frameError.resize(perFrame.offset()+1, perFrame.size());
    for (int j = perFrame.firstIndex(); j <= perFrame.lastIndex(); ++j) {
      t->data.frameError[j+1] = robustWeight(perFrame[j], kIrls_t2);
    }
  }

  //// compute 2nd eval per frame
  //ofstream evalOut(stackOutFn_);
  //evalOut << inFns_[0] << ",,";

  //double minEval = 1.0;
  //for (int f = bestModel_[modelNum]->firstFrame();
       //f < bestModel_[modelNum]->lastFrame(); ++f) {
    //Matx33d F = bestModel_[modelNum]->getTransform(f);

    //Vec3d w;
    //Matx33d U, Vt;
    //SVD::compute(F, w, U, Vt);

    ////minEval = std::min(w[1], minEval);

    //evalOut << w[1] << ",";

  //}
  //evalOut << endl;
  //evalOut.close();
}

// get all tracks that are outliers wrt models up to (not including) modelNum
template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::getOutlierTracks(int modelNum,
                                                 vector<PTrack> &tracks) {
  // select tracks that are not inliers to any model prior to modelNum
  for (auto &&t : inputTracks_) {
    bool outlier = true;
    for (int mI = 0; mI < modelNum; ++mI) {
      outlier = outlier && (t->data.inlWts[mI] < 0.5);
    }
    if (outlier) {
      tracks.push_back(t);
    }
  }
}

// get all tracks that are inliers wrt modelNum
template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::getInlierTracks(int modelNum,
                                                vector<PTrack> &tracks) {
  for (auto &&t : inputTracks_) {
    bool skip = false;
    if (t->data.inlWts[modelNum] > 0.5) {
      for (int mI = 0; mI < kNumModels; ++mI) {
        if (t->data.inlWts[mI] > t->data.inlWts[modelNum]) {
          skip = true;
        }
      }
      if (!skip) {
        tracks.push_back(t);
      }
    }
  }
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::initModelTrackwise(
    const vector<PTrack>& initTracks, const EstT &estimator) {
  // setup ransac machinery
  theia::RansacSummary summary;

  // initialize the model with a prosac fit, sorting tracks by length
  std::unique_ptr<theia::SampleConsensusEstimator<EstT>> sac(
      new theia::Prosac<EstT>(params_, estimator));

  auto trackCmp = [initTracks](size_t i1, size_t i2) {
    return initTracks[i1]->length() > initTracks[i2]->length();
  };
  vector<size_t> sortedIndices(initTracks.size());
  iota(sortedIndices.begin(), sortedIndices.end(), 0);
  sort(sortedIndices.begin(), sortedIndices.end(), trackCmp);

  vector<PTrack> sortedInput(initTracks.size());
  for (int i = 0; i < initTracks.size(); ++i) {
    sortedInput[i] = initTracks[sortedIndices[i]];
  }

  sac->Initialize();
  sac->Estimate(sortedInput, &bestModel_.back(), &summary);
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::initModelFramewise(
    const vector<PTrack>& initTracks, const EstT &estimator) {
  // create and initialize model
  bestModel_.back().reset(new ModelT(height_, width_));
  bestModel_.back()->init(initTracks, initTracks.size(),
                          bestModel_.back()->minSetSize());

  shared_ptr<ModelT>& model = bestModel_.back();

  vector<Vec2f> obs1, obs2;
  for (int fI = model->firstFrame(); fI < model->lastFrame(); ++fI) {
    obs1.clear();
    obs2.clear();
    for (const PTrack& t : initTracks) {
    //auto fTracks = tracks_.frameTracks(fI);
    //for (auto it = fTracks.begin(); it != fTracks.end(); ++it) {
      if (t->inFrame(fI) && t->inFrame(fI + 1)) {
        obs1.push_back(t->obs(fI).loc);
        obs2.push_back(t->obs(fI + 1).loc);
      }
    }
    Matx33f transform = model->estimateTransformRobust(obs1, obs2);
    model->setTransform(transform, fI);
    //cout << fI << " " << bestModel_.back()->getConfidence(fI) << " " << transform << endl;
  }
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::estimateIrls(vector<PTrack>& tracks,
                                             vector<float> &weights,
                                             const EstT &estimator, int maxIter,
                                             double t2) {
  // if no initial weights given, set all to 1
  if (weights.size() != tracks.size()) {
    cout << "Starting with uniform weights." << endl;
    weights.clear();
    weights.assign(tracks.size(), 1.0);
  }
  vector<float> bestWeights(weights.size(), 1.0);
  vector<float> inputWeights(weights.size(), 1.0);
  vector<float> outputWeights(weights.size(), 1.0);

  inputWeights.swap(weights); // inputWeights <- weights;

  double bestCost = -1;
  double cost = -1;
  double stepSize = 1.0;

  //PModelT model;
  shared_ptr<ModelT> model;
  for (int itNum = 0; itNum < maxIter; ++itNum) {
    // fit a model

    model.reset(new ModelT());
    model->fromTracks(tracks, tracks.size(), inputWeights);

    // compute its induced weights and cost
    cost = 0;
    for (int trackI = 0; trackI < tracks.size(); ++trackI) {
      double d = estimator.Error(tracks[trackI], model);
      double d2 = pow(d, 2);
      double w2 = robustWeight(d2, t2);
      outputWeights[trackI] =
          (1 - stepSize) * bestWeights[trackI] + stepSize * w2;
      cost += robustCost(d2, t2);
    }
    cout << "Iteration " << itNum << " complete with cost " << cost;

    if (cost > bestCost && bestCost >= 0) {
      // it got worse
      stepSize /= 4.0;
      cout << " (reducing step size to " << stepSize << ")";

    } else if (cost <= bestCost || bestCost < 0) {
      // it got better
      cout << " (improvement!)";
      stepSize = std::min(4.0 * stepSize, 1.0);
      for (int i = 0; i < bestWeights.size(); ++i) {
        bestWeights[i] = inputWeights[i];
      }
      if (cost > 0 &&
          std::abs((cost - bestCost) / bestCost) < kIrls_Threshold) {
        cout << " (convergence reached)" << endl;
        break;
      }
      bestCost = cost;
      bestModel_.back() = model;
    }
    cout << endl;
    inputWeights.swap(outputWeights);
  }
  weights.swap(bestWeights);
}

//template <class EstT, class ModelT>
//void TrackRansac<EstT, ModelT>::compareModelsDirect(int m1, int m2) {
  //int first = max(bestModel_[m1]->firstFrame(), bestModel_[m2]->firstFrame());
  //int last = min(bestModel_[m1]->lastFrame(), bestModel_[m2]->lastFrame());
  //for (int frameI = first; frameI < last; ++frameI) {
    //Matx33f H1 = bestModel_[m1]->getTransform(frameI);
    //Matx33f H2 = bestModel_[m2]->getTransform(frameI);

    //Matx33f H12;
    //solve(H1.t(), H2.t(), H12);
    //H12 = H12.t();

    //Vec3f sigma;
    //SVD::compute(H12 - Matx33f::eye(), sigma, SVD::NO_UV);
    //cout << sigma.t() << endl;
  //}

//}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::compareModels(int m1, int m2) {
  // collect inliers to m2
  vector<PTrack> m2Inl;
  getInlierTracks(m2, m2Inl);

  // homogeneous position in image 1 and H1*position in image 2
  vector<Vec3f> p1, p2, p2Act;
  int first = max(bestModel_[m1]->firstFrame(), bestModel_[m2]->firstFrame());
  int last = min(bestModel_[m1]->lastFrame(), bestModel_[m2]->lastFrame());
  for (int frameI = first; frameI < last; ++frameI) {
    p1.clear(); p2.clear(); p2Act.clear();
    if (bestModel_[m1]->getConfidence(frameI) == 0 ||
        bestModel_[m2]->getConfidence(frameI) == 0) {
      continue;
    }
    for (PTrack &t : m2Inl) {
      if (t->inFrame(frameI) && t->inFrame(frameI+1)) {
        p1.push_back(homog(t->obs(frameI).loc));
        p2.push_back(homog(bestModel_[m1]->warpPt(t, frameI, frameI+1)));
        p2Act.push_back(homog(t->obs(frameI+1).loc));
      }
    }
    if (p1.size() < 3) {
      continue;
    }
    // find the epipole of the homography from p2Act to p2
    Matx33f CtC = Matx33f::zeros();
    for (int tI = 0; tI < p2.size(); ++tI) {
       Vec3f ci = p2[tI].cross(p2Act[tI]);
       CtC += (ci * ci.t());
    }
    // compute nullspace (smallest eigenvector of CtC)
    Mat evals;
    float evecs[3][3];
    Mat matEvecs(3, 3, CV_32F, &evecs[0][0]);
    eigen(CtC, evals, matEvecs);
    Vec3f epipole(evecs[2]);

    Mat p2Mat(p2.size(), 3, CV_32F);
    vector<float> zVals(p2.size());
    // find pseudo-depth for each track
    Matx<float, 3, 2> A;
    Vec2f x;
    for (int tI = 0; tI < p2.size(); ++tI) {
      hconcat(p2[tI], epipole, A);
      solve(A, p2Act[tI], x, CV_QR);
      zVals[tI] = x(0) / x(1);

      // fill in p2Mat
      float* p2mPtr = p2Mat.ptr<float>(tI);
      p2mPtr[0] = p2[tI](0);
      p2mPtr[1] = p2[tI](1);
      p2mPtr[2] = p2[tI](2);
    } // tI

    cout << p2Mat.rows << "x" << p2Mat.cols << " " << zVals.size() << endl;
    // fit ax + by + c= z
    Mat abc(3,1,CV_32F);
    Mat zMat(zVals);
    solve(p2Mat, zMat, abc, CV_QR);

    Mat residMat = abs((p2Mat * abc) - zMat);
    float* rmPtr = residMat.ptr<float>();
    vector<float> resid(rmPtr, rmPtr + residMat.rows);

    // do some visualization
    vector<Vec2f> actual;
    vector<pair<Vec2f, bool>> predicted;
    Mat3b frame = imread(inFns_[frameI+1]);

    // calculate mean, max, min
    float mx = 0, mn = 1e10, mean = 0;
    for (int tI = 0; tI < p2.size(); ++tI) {
      mean += resid[tI];
      mx = std::max(mx, resid[tI]);
      mn = std::min(mn, resid[tI]);
    }

    double zmin, zmax;
    minMaxLoc(zMat, &zmin, &zmax);

    vector<float> zDisp(zVals.size());
    float var = 0;
    for (int tI = 0; tI < p2.size(); ++tI) {
      zDisp[tI] = (zVals[tI] - zmin) / (zmax - zmin);
      var += pow(resid[tI] - mean, 2);
      //zVals[tI] = (zVals[tI] - mn) / (mx - mn);
      //resid[tI] /= 3;
      resid[tI] = std::min(resid[tI], 1.0f);
      predicted.push_back(make_pair(dehomog(p2[tI]) + offset_, true));
      actual.push_back(dehomog(p2Act[tI]) + offset_);
    }
    var /= (p2.size() - 1);
    draw(frame, actual, predicted, resid, COLORMAP_WINTER);
    cout << frameI << " " << mn << " " << mx << " " << sqrt(var) << " "
         << abc.t() << endl;

    char outfn[1024];
    string suffix = "hc" + to_string(m1) + to_string(m2);
    sprintf(outfn, visFormat_.c_str(), suffix.c_str(), frameI);
    imwrite(outfn, frame);

    frame = imread(inFns_[frameI+1]);
    draw(frame, actual, predicted, zDisp, COLORMAP_SUMMER);
    suffix = "hcz" + to_string(m1) + to_string(m2);
    sprintf(outfn, visFormat_.c_str(), suffix.c_str(), frameI);
    imwrite(outfn, frame);
  } // frameI
  cout << endl;
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::writeOverlayFrames(const string& suffix) {
  // load fresh input frames
  frames_.resize(inFns_.size());
  for (int i = 0; i < inFns_.size(); ++i) {
    frames_[i] = imread(inFns_[i]);
  }

  // draw overlays on the frames and save out images
  for (int frameI = tracks_.startFrame() + 1; frameI <= tracks_.lastFrame();
       ++frameI) {
    // compute actual and predicted position for each point

    // keep a separate vector for each model so we can colormap them separately
    // because std::optional isn't here yet:
    // predicted.first stores the value, predicted.second indicates presence
    vector<vector<pair<Vec2f, bool>>> predicted(kNumModels);
    vector<vector<Vec2f>> actual(kNumModels);
    vector<vector<float>> trackWiseErrors(kNumModels);

    for (int trackI = 0; trackI < inputTracks_.size(); ++trackI) {
      const PTrack t = inputTracks_[trackI];
      if (t->length() < kMinOverlap || !t->inFrame(frameI)) {
        continue;
      }

      // find which model this track fits best
      int bestModelI = 0;
      for (int mI = 1; mI < kNumModels; ++mI) {
        if (t->data.inlWts[mI] > t->data.inlWts[bestModelI]) {
          bestModelI = mI;
        }
      }

      actual[bestModelI].push_back(t->obs(frameI).loc + offset_);
      shared_ptr<ModelT>& model = bestModel_[bestModelI];
      // fill in a predicted value if available
      if (model->firstFrame() < frameI &&
          frameI <= model->lastFrame() &&
          t->inFrame(frameI - 1) &&
          model->getConfidence(frameI-1) > 0) {

        predicted[bestModelI].push_back(
            make_pair(model->warpPt(t, frameI - 1, frameI) + offset_, true));
      } else {
        predicted[bestModelI].push_back(make_pair(Vec2f(0,0), false));
      }

      if (!visPerFrame_) {
        // fill in trackwise inlier weight
        trackWiseErrors[bestModelI].push_back(t->data.inlWts[bestModelI]);
      } else {
        // fill in the inlier weight of this particular correspnodence
        if (t->data.frameError.hasIndex(frameI)) {
          trackWiseErrors[bestModelI].push_back(t->data.frameError[frameI]);
        }
      }
    } // track
    for (int mI = 0; mI < kNumModels; ++mI) {
      int cmap = kColorMaps[mI];
      draw(frames_[frameI], actual[mI], predicted[mI], trackWiseErrors[mI],
           cmap);
    }

    char fn[1024];
    sprintf(fn, visFormat_.c_str(), suffix.c_str(), frameI);
    imwrite(fn, frames_[frameI]);
  } // frame
}

template <class EstT, class ModelT>
TrackRansac<EstT, ModelT>::TrackRansac(
    const string& trackFn,
    const string& inDir,
    const theia::RansacParameters& params,
    const string& stackOutFn,
    const string& visFormat,
    const string& trackOutFn,
    const bool visPerFrame)
    : params_(params),
      stackOutFn_(stackOutFn),
      visFormat_(visFormat),
      trackOutFn_(trackOutFn),
      visPerFrame_(visPerFrame) {
  // load tracks from file
  FILE* infile = fopen(trackFn.c_str(), "r");
  tracks_.deserialize(infile);
  fclose(infile);

  //inFns_.resize(tracks_.lastFrame() + 1);
  ifstream bmffile(inDir+ "/images.bmf");
  int nFrames;
  bmffile >> nFrames;
  string tmp;
  getline(bmffile, tmp);
  for (int i = 0; i < nFrames; ++i) {
    string fn;
    getline(bmffile, fn);
    inFns_.push_back(inDir+ "/" + fn);
  }


  //for (int i = tracks_.startFrame(); i <= tracks_.lastFrame(); ++i) {
    //char fn[1024];
    //sprintf(fn, inFormat.c_str(), i);
    //inFns_[i] = fn;
  //}
  Mat im0 = imread(inFns_[0]);
  width_ = im0.cols;
  height_ = im0.rows;
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::writeTrackFile(string suffix) {
  FILE* fout = fopen((trackOutFn_ + suffix + ".tt").c_str(), "w");
  tracks_.serialize(fout);
  fclose(fout);
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::writeStackFile(const string& suffix) {
  ofstream outfile((stackOutFn_ + suffix + ".txt").c_str());
  ofstream outfilem((stackOutFn_ + suffix + "m.txt").c_str());
  for (int f = bestModel_[0]->firstFrame(); f < bestModel_[0]->lastFrame();
       ++f) {
    outfile << "Frame " << f << endl;
    outfilem << f << endl;
    for (int m = 0; m < bestModel_.size(); ++m) {
      outfile << "Model " << m;
      outfilem << m << " ";
      if (f >= bestModel_[m]->firstFrame() && f < bestModel_[m]->lastFrame()) {
        outfile << " Confidence " << bestModel_[m]->getConfidence(f) << endl;
        outfilem << bestModel_[m]->getConfidence(f);

        Matx33f H = bestModel_[m]->getTransform(f);
        outfile << bestModel_[m]->getTransform(f) << endl << endl;
        for (int hi = 0; hi < 3; ++hi) {
        for (int hj = 0; hj < 3; ++hj) {
          outfilem << " " << H(hi,hj);
        }
        }
        outfilem << endl;
      }
    }
  }
}

template <class EstT, class ModelT>
void TrackRansac<EstT, ModelT>::run() {
  preprocessTracks();
  for (int i = 0; i < kNumModels; ++i) {
    estimateModel(i);
  }

  //compareModels(0,1);
  //compareModelsDirect(0,1);

  // write out a stack file for viewing in basicviewer
  if (stackOutFn_ != "") {
    writeStackFile("final");
  }

  // write out frames with tracks overlaid
  if (visFormat_ != "") {
    writeOverlayFrames("final");
  }

  // write out a serialized tracktable file with populated RansacData
  if (trackOutFn_ != "") {
    writeTrackFile("final");
  }
}

int main(int argc, char** argv) {
  // Parse arguments
  CommandLineParser parser(
      argc, argv,
      "{help h        || print this message }"
      "{@infile       || input track filename }"
      "{@indir        || input directory with images and images.bmf}"
      "{model m       || motion model (translation, rotation3d, homography)}"
      "{visformat v   || printf-style format string for track vis filenames}"
      "{visperframe f || do error visualization per frame, not per track}"
      "{trackout t    || output track filename }"
      "{stackout s    || output ImageStack filename}");

  if (parser.has("help") || argc == 1) {
    parser.printMessage();
    return 0;
  }
  if (!parser.check()) {
    parser.printErrors();
    return 1;
  }

  string inFn = parser.get<string>("@infile");
  string inDir = parser.get<string>("@indir");
  string modelName = parser.get<string>("model");
  string stackOutFn = parser.get<string>("stackout");
  string visFormat = parser.get<string>("visformat");
  string trackOutFn = parser.get<string>("trackout");
  bool visPerFrame = parser.has("visperframe");


  // Set the ransac parameters.
  theia::RansacParameters params;
  params.error_thresh = kRansacErrorThresh;
  params.min_inlier_ratio = 0.1;
  params.min_iterations = kRansacIter;
  params.max_iterations = kRansacIter;

  if (modelName == "homography") {
    TrackRansac<TransformEstimator<shared_ptr<Homography>>, Homography> tr(
        inFn, inDir, params, stackOutFn, visFormat, trackOutFn, visPerFrame);
    tr.run();
  } else {
    cout << "Unknown model type." << endl;
    return 1;
  }
  return 0;
}
