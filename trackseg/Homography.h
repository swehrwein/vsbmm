/*
  Contains Homography, which is a class representing a homography motion model.
  The model keeps vector of transforms and can chain them together to warp
  points get from one frame to another.

*/
#pragma once

#include "MotionModel.h"

#include <opencv2/core/core.hpp>

#include "../common/TrackTable.h"

#include "TransformEstimator.h"
#include "TrackRansac.h"

class Homography : public MotionModel {
  friend class TransformEstimator<std::shared_ptr<Homography>>;

 protected:
  // tCumulative[i] stores the homography to get from firstFrame to firstFrame+i
  OffsetVector<cv::Matx33f> tCumulative_;

 public:
  Homography() {}
  Homography(const int height, const int width) : MotionModel(height, width) {}
  cv::Matx33f getTransform(int frame) const;

  // given a point pt in fromFrame, give its predicted coordintes in toFrame
  cv::Vec2f warpPt(const PTrack &pt, int fromFrame, int toFrame) const;

  bool getResiduals(const PTrack &track,
                    OffsetVector<double> &residuals) const override;

  // size of the minimum set needed to estimate a homography
  // this is nonstatic so it can be overridden by more specialized motion
  // models
  const virtual int minSetSize() const override { return 4; }

  // estimate a series of transformations using any number of tracks
  virtual void fromTracks(
      const std::vector<PTrack> &tracks, const int nTracks,
      const std::vector<float> &weights = std::vector<float>()) override;

  // estimate a pairwise transfrom given points observed in two images
  virtual cv::Matx33f estimateTransform(
      const std::vector<cv::Vec2f> &img1Obs,
      const std::vector<cv::Vec2f> &img2Obs,
      const std::vector<float> &weights = std::vector<float>()) override;

  virtual cv::Matx33f
  estimateTransformRobust(const std::vector<cv::Vec2f> &img1Obs,
                          const std::vector<cv::Vec2f> &img2Obs) override;

  virtual float estimateConfidence(const std::vector<cv::Vec2f> &img1Obs,
                                   const std::vector<cv::Vec2f> &img2Obs,
                                   const cv::Matx33f &transform) override;

  cv::Matx33f weightedFindHomography(const std::vector<cv::Vec2f> &img1Obs,
                                     const std::vector<cv::Vec2f> &img2Obs,
                                     const std::vector<float> &weights);
};

typedef std::shared_ptr<Homography> PHomography;

