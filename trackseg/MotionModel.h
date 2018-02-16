/*
  abstract base class for motion models to be used with
  TrackRansac framework
*/
#pragma once

#include <opencv2/core/core.hpp>

#include <TrackRansac.h>

class MotionModel {
  // Base class for homography-based motion models used with
  // Theia's RANSAC library
 protected:
  // data members
  int firstFrame_, lastFrame_;
  int height_, width_;

  // t_[i] is the transform matrix from frame i to frame i+1
  OffsetVector<cv::Matx33f> t_;

  // confidence_[i] is the confidence in t_[i]
  OffsetVector<float> confidence_;

 public:
  int firstFrame() const {
    return firstFrame_;
  }
  virtual int lastFrame() const {
    return lastFrame_;
  }
  MotionModel() {}
  MotionModel(const int height, const int width)
      : height_(height), width_(width) {}

  double medianResidual;

  float getConfidence(int frame);

  virtual bool getResiduals(
      const PTrack& track,
      OffsetVector<double>& residuals) const = 0;

  // size of the minimum set needed to estimate the model
  // this is nonstatic so it can be overridden by each specialized motion models
  const virtual int minSetSize() const = 0;

  // compute the overlap among a set of tracks.
  // sets firstFrame_ and lastFrame_ to be the first and last frame such that
  // among tracks[0..nTracks], at least minTracks are observed.
  //
  // note that this does not ensure that minTracks are observed in all frames
  // between, so it's up to the derived class to handle gaps
  // returns false if overlap is smaller than kMinOverlap
  // (defined in TrackRansac.h)
  bool init(
      const std::vector<PTrack>& tracks,
      const int nTracks,
      const int minTracks);

  // estimate a series of transformations using any number of tracks
  virtual void fromTracks(
      const std::vector<PTrack>& tracks,
      const int nTracks,
      const std::vector<float>& weights);

  // estimate a pairwise transfrom given points observed in two images
  virtual cv::Matx33f estimateTransform(
      const std::vector<cv::Vec2f> &img1Obs,
      const std::vector<cv::Vec2f> &img2Obs,
      const std::vector<float> &weights = std::vector<float>()) = 0;

  virtual cv::Matx33f
  estimateTransformRobust(const std::vector<cv::Vec2f> &img1Obs,
                          const std::vector<cv::Vec2f> &img2Obs) = 0;

  virtual float estimateConfidence(const std::vector<cv::Vec2f> &img1Obs,
                                   const std::vector<cv::Vec2f> &img2Obs,
                                   const cv::Matx33f &transform) = 0;

  void setTransform(cv::Matx33f transform, int frameIndex);

  // normalize correspondences and calculate the the pre- and post-multiplied
  // normalization transforms
  void normalizeCorrespondences(
      const std::vector<cv::Vec2f>& img1Obs,
      const std::vector<cv::Vec2f>& img2Obs,
      std::vector<cv::Vec2f>& img1Norm,
      std::vector<cv::Vec2f>& img2Norm,
      cv::Vec2f& cent1,
      cv::Vec2f& cent2,
      cv::Vec2f& scale1,
      cv::Vec2f& scale2);

};

