/*
  Contains RansacData, a struct that stores inlier bookkeeping
  information in Tracks in a TrackTable.

  TrackRansac.cpp uses Theia's RANSAC library to robustly estimate background
  motion using one of several motion models (e.g., Translation.h).

*/
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <theia/theia.h>

//#include <../common/Util.h>
#include <../common/RansacData.h>
#include <../common/FileIo.h>
#include <../common/OffsetVector.h>
#include <../common/TrackTable.h>

enum InitMethod { TRACKWISE, FRAMEWISE };

// arbitragic constants:
const InitMethod kInitMethod = FRAMEWISE; // initialization method
const int kMinOverlap = 5; // min track overlap/length to consider
const double kRansacErrorThresh = 2.0; // ransac error threshold
const int kRansacIter = 200; // number of ransac iterations
// unused const double kOutlierWeight = 0.5; // initial weight for initialization outliers
const double kIrls_t2 = 16.0; // squared inlier threshold for IRLS
const double kIrls_Threshold = 1e-3; // declare victory if rel err < this
const int kColorMaps[] = {cv::COLORMAP_WINTER,  // blue -- green
                          cv::COLORMAP_SPRING,  // pink -- yellow
                          cv::COLORMAP_BONE};   // black -- white
const int kNumModels = 1;

//const float kPNorm = 8.0; // p for p-norm cost over track errors
const float kPNorm = -1; // use the max over track errors

const double default_t2 = 4.0;

cv::Vec3f homog(const cv::Vec2f x);

cv::Vec2f dehomog(const cv::Vec3f x);

typedef std::shared_ptr<Track<Observation2d, RansacData>> PTrack;

template <class EstT, class ModelT>
class TrackRansac {
 protected:
  TrackTable<Observation2d, RansacData> tracks_;
  std::vector<PTrack> inputTracks_;
  std::vector<int> trackModel_; // which model a given track belongs to
  std::vector<std::string> inFns_;
  std::vector<cv::Mat3b> frames_;
  theia::RansacParameters params_;
  int width_;
  int height_;
  std::string stackOutFn_;
  std::string visFormat_;
  std::string trackOutFn_;
  bool visPerFrame_;
  cv::Vec2f offset_;

  std::vector<std::shared_ptr<ModelT>> bestModel_;

 public:
  // construct TrackRansac object
  TrackRansac(
      const std::string& trackFn,
      const std::string& inFormat,
      const theia::RansacParameters& params,
      const std::string& stackOutFn,
      const std::string& visFormat,
      const std::string& trackOutFn,
      const bool visPerFrame);

  // offset tracks to middle-origin coordinate system and filter short tracks
  void preprocessTracks();

  void getOutlierTracks(int modelNum, std::vector<PTrack>& tracks);
  void getInlierTracks(int modelNum, std::vector<PTrack>& tracks);
  void initModelTrackwise(const std::vector<PTrack>& initTracks,
                          const EstT &estimator);
  void initModelFramewise(const std::vector<PTrack>& initTracks,
                          const EstT &estimator);

  // Run ransac/IRLS for the tracks in a given table, setting bestModel_
  void estimateModel(const int modelNum);

  // Run IRLS refinement given initial weights
  void estimateIrls(std::vector<PTrack>& tracks, std::vector<float> &weights,
                    const EstT &Estimator, int maxIter, double t2);

  void compareModels(int m1, int m2);
  void compareModelsDirect(int m1, int m2);
  // Write the resulting transformation information out to an ImageStack format
  // for viewing in basicviewer
  void writeStackFile(const std::string& suffix);

  // save frames with overlaid dots visualizing inlier and outlier tracks
  void writeOverlayFrames(const std::string& suffix);

  // save out a serialized tracktable file
  void writeTrackFile(std::string suffix);

  // run all of the above where applicable
  void run();

  double robustWeight(double residual, double t2 = default_t2) const {
    return std::max(0.0, 1.0 - residual * residual / t2);
  }

  double robustCost(double residual, double t2 = default_t2) const {
    float r2 = residual * residual;
    return (r2 < t2) ? r2 * (1 - (r2 / 2 / t2)) : t2 / 4;
    //return w2 * d2 + t2 * pow(1 - w2, 2) / 2;
  };

  void evalModel(const EstT& estimator, ModelT& model, std::vector<float>& weights, double& cost);
    //for (int trackI = 0; trackI < inputTracks_.size(); ++trackI) {
      //double d = estimator.Error(inputTracks_[trackI], model);
      //double d2 = pow(d, 2);
      //double w2 = robustWeight(d2, t2);
      //outputWeights[trackI] =
          //(1 - stepSize) * bestWeights[trackI] + stepSize * sqrt(w2);
      //cost += robustCost(d2, t2);
    //}
};
