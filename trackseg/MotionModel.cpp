#include "MotionModel.h"

#include "../common/TrackTable.h"

#include "TrackRansac.h"

using namespace cv;
using namespace std;

using std::abs;

//// MotionModel Implementation
bool MotionModel::init(
    const vector<PTrack>& tracks,
    const int nTracks,
    const int minTracks) {
  // find the first and last frame with any tracks at all
  int first = tracks[0]->firstFrame();
  int last = tracks[0]->lastFrame();
  for (int i = 0; i < nTracks; ++i) {
    first = std::min(first, tracks[i]->firstFrame());
    last = std::max(last, tracks[i]->lastFrame());
  }

  // count tracks seen per frame
  OffsetVector<int> counts(first, last - first + 1);
  for (int i = 0; i < nTracks; ++i) {
    for (int frame = first; frame <= last; ++frame) {
      if (tracks[i]->inFrame(frame)) {
        ++counts[frame];
      }
    }
  }

  // find the first and last frame that sees >= minTracks tracks
  // there may be gaps; these are to be handled in the motion models
  firstFrame_ = last;
  lastFrame_ = first;
  for (int frame = first; frame <= last; ++frame) {
    if (counts[frame] >= minTracks) {
      firstFrame_ = std::min(firstFrame_, frame);
      lastFrame_ = std::max(lastFrame_, frame);
    }
  }
  if (lastFrame_ - firstFrame_ < kMinOverlap) {
    return false;
  }
  t_.clear();
  t_.resize(firstFrame(), lastFrame() - firstFrame());
  confidence_.clear();
  confidence_.resize(firstFrame(), lastFrame() - firstFrame());
  for (int i = confidence_.firstIndex(); i <= confidence_.lastIndex(); ++i) {
    confidence_[i] = 1.0;
  }
  return true;
}

float MotionModel::getConfidence(int frame) {
  assert(confidence_.hasIndex(frame));
  return confidence_[frame];
}

void MotionModel::setTransform(cv::Matx33f transform, int frameIndex) {
  if (!t_.hasIndex(frameIndex)) {
    cout << "setting transform for invalid frame.";
  }
  t_[frameIndex] = transform;
}

void MotionModel::fromTracks(
    const vector<PTrack>& tracks,
    const int nTracks,
    const vector<float>& sqWeights) {
  // sets first and last frames; inherited from MotionModel
  assert(nTracks <= tracks.size());
  assert(sqWeights.size() == nTracks);

  init(tracks, nTracks, minSetSize());

  t_.clear();
  t_.resize(firstFrame(), lastFrame() - firstFrame());

  vector<Vec2f> frame1Obs;
  vector<Vec2f> frame2Obs;
  vector<float> frameWeights;
  //cout << "Skipped frames: [";
  for (int frame = firstFrame(); frame < lastFrame(); ++frame) {
    frame1Obs.clear();
    frame2Obs.clear();
    frameWeights.clear();
    float weightSum = 0.0;
    for (int i = 0; i < nTracks; ++i) {
      if (tracks[i]->inFrame(frame) && tracks[i]->inFrame(frame + 1)) {
        frame1Obs.push_back(tracks[i]->obs(frame).loc);
        frame2Obs.push_back(tracks[i]->obs(frame + 1).loc);
        frameWeights.push_back(sqWeights[i]);
        weightSum += sqWeights[i];
      }
    }
    if (frame1Obs.size() >= minSetSize() && weightSum > 1.0) {
      t_[frame] = estimateTransform(frame1Obs, frame2Obs, frameWeights);
      confidence_[frame] = estimateConfidence(frame1Obs, frame2Obs, t_[frame]);
    } else {
      //cout << frame << " ";
      t_[frame] = Matx33f::eye(); // not enough samples, punt and set identity
      confidence_[frame] = 0;
    }
  }
  //cout << "]" << endl;
}

// return centroids and scale factors for each image so that a transform can
// be estimated on well-conditioned point values. the estiamted transform must
// then be amended with the proper normalizing and un-normalizing
// transformations.
void MotionModel::normalizeCorrespondences(
    const vector<Vec2f>& img1Obs,
    const vector<Vec2f>& img2Obs,
    vector<Vec2f>& img1Norm,
    vector<Vec2f>& img2Norm,
    Vec2f& cent1,
    Vec2f& cent2,
    Vec2f& scale1,
    Vec2f& scale2) {
  const int n = img1Obs.size();

  img1Norm.clear();
  img1Norm.resize(n);
  img2Norm.clear();
  img2Norm.resize(n);

  // compute centroids
  cent1(0) = 0;
  cent1(1) = 0;
  cent2(0) = 0;
  cent2(1) = 0;
  for (int i = 0; i < n; ++i) {
    cent1 += img1Obs[i];
    cent2 += img2Obs[i];
  }
  cent1 /= n;
  cent2 /= n;

  // compute a scale per image (isotropic in x,y)
  scale1(0) = 0;
  scale1(1) = 0;
  scale2(0) = 0;
  scale2(1) = 0;
  for (int i = 0; i < n; ++i) {
    float s1 = norm(img1Obs[i] - cent1);
    scale1(0) += s1;
    scale1(1) += s1;
    float s2 = norm(img2Obs[i] - cent2);
    scale2(0) += s2;
    scale2(1) += s2;
  }
  // normalize by n s.t. avg distance to origin == sqrt(2)
  float normConst = sqrt(2) * n;
  scale1(0) = normConst / scale1(0);
  scale1(1) = normConst / scale1(1);
  scale2(0) = normConst / scale2(0);
  scale2(1) = normConst / scale2(1);

  // fill in normalized correspondence vectors
  for (int i = 0; i < n; ++i) {
    img1Norm[i](0) = (img1Obs[i](0) - cent1(0)) * scale1(0);
    img1Norm[i](1) = (img1Obs[i](1) - cent1(1)) * scale1(1);
    img2Norm[i](0) = (img2Obs[i](0) - cent2(0)) * scale2(0);
    img2Norm[i](1) = (img2Obs[i](1) - cent2(1)) * scale2(1);
  }
}

