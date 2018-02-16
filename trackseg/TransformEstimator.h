/*
 * Theia-derived estimator class for track ransac
 */
#pragma once

#include <theia/theia.h>


template <class PModelT>
class TransformEstimator : public theia::Estimator<PTrack, PModelT> {
  // Robust estimator for homography-based motion models.
  // see docs for theia::Estimator
 public:
  // returns minimal set size by getting it from a PModelT instance
  virtual double SampleSize() const;

  virtual bool EstimateModel(
      const std::vector<PTrack>& data,
      std::vector<PModelT>* models) const;

  //virtual bool estimateIrls(
      //const std::vector<PTrack>& data,
      //std::vector<float>& weights,
      //PModelT& bestModel,
      //int maxIter = 50,
      //double t2 = DEFAULT_T2) const;

  // returns max reprojection error over all frames
  double Error(const PTrack& track, const PModelT& model) const;

};
