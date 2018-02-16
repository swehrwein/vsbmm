/*
TransformEstimator-inl.h - templated implementation of TransformEstimator
*/
#pragma once

#include "../common/TrackTable.h"

#include "TrackRansac.h"

//// TransformEstimator implementation
template <class PModelT>
double TransformEstimator<PModelT>::SampleSize() const {
  // faking static polymorphism: make an instance and ask its set size.
  // seemed cleaner than adding more layers of templates (a la CRTP)
  PModelT dummyModel(new typename PModelT::element_type);
  return dummyModel->minSetSize();
}

template <class PModelT>
bool TransformEstimator<PModelT>::EstimateModel(
    const std::vector<PTrack>& data,
    std::vector<PModelT>* models) const {
  PModelT model(new typename PModelT::element_type);

  // check for sufficient overlap
  if (!model->init(data, data.size(), model->minSetSize())) {
    return false;
  }

  std::vector<float> weights(data.size(), 1.0);
  // estimate transform from a minimal set
  model->fromTracks(data, SampleSize(), weights);
  models->push_back(model);
  return true;
}

template <class PModelT>
double TransformEstimator<PModelT>::Error(
    const PTrack& track,
    const PModelT& model) const {
  // Compute error between a model and a track

  OffsetVector<double> residuals;
  bool validOverlap = model->getResiduals(track, residuals);

  double result = 0.0;

  if (kPNorm >= 0) {
    // p-norm or per-frame residuals is the track residual
    for (int frameI = residuals.firstIndex(); frameI <= residuals.lastIndex();
         ++frameI) {
      result += pow(residuals[frameI], kPNorm) * model->getConfidence(frameI);
    }
    result = pow(result, 1.0 / kPNorm);
  } else {
    result = 0;
    for (int frameI = residuals.firstIndex(); frameI <= residuals.lastIndex();
        ++frameI) {
      result = std::max(result, residuals[frameI]);
    }
  }

  if (validOverlap) {
    return result;
  } else {
    return 1e5;
  }
}
