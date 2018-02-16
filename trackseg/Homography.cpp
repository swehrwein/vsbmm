#include "Homography.h"

#include "TrackRansac.h"

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

//// Homography implementation
cv::Matx33f Homography::getTransform(int frame) const {
  return t_[frame];
}

Vec2f Homography::warpPt(const PTrack& track, int fromFrame, int toFrame)
    const {
  assert(fromFrame >= firstFrame() && toFrame <= lastFrame() + 1);

  // chain transforms
  Vec2f pt = track->obs(fromFrame).loc;
  Vec3f result(pt(0), pt(1), 1);
  for (int i = toFrame - 1; i >= fromFrame; --i) {
    result = t_[i] * result;
  }

  // apply to homogeneous pt and dehomogenize
  return Vec2f(result(0) / result(2), result(1) / result(2));
}

// return true if overlap is large enough, false otherwise
bool Homography::getResiduals(
    const PTrack& track,
    OffsetVector<double>& residuals) const {
  int overlapFirst = std::max(firstFrame(), track->firstFrame());
  int overlapLast = std::min(lastFrame(), track->lastFrame());
  //cout << "overlap " << endl;
  //cout << firstFrame() << " " << track->firstFrame() << endl;
  //cout << lastFrame() << " " << track->lastFrame() << endl;
  //cout << overlapFirst << " " << overlapLast << endl;

  // residuals[i] is the residual of the point's predicted location in frame i+1
  residuals.resize(overlapFirst, std::max(overlapLast - overlapFirst, 0));

  // last frame has no prediction
  //if (residuals.size() > 0) {
    //residuals[overlapFirst] = 0;
  //}
  for (int i = overlapFirst; i < overlapLast; ++i) {
    // warp the point from frame i to i+1
    cv::Vec2f pt2 = track->obs(i).loc;
    cv::Vec2f pt2Warped = warpPt(track, i, i+1);

    // compare to its actual location in frame i+1
    cv::Vec2f pt2Actual = track->obs(i+1).loc;
    residuals[i] = norm(pt2Warped, pt2Actual);
  }
  // return true if overlap is large enough, false otherwise
  return overlapLast - overlapFirst >= kMinOverlap;
}

void Homography::fromTracks(
    const vector<PTrack>& tracks,
    const int nTracks,
    const vector<float>& sqWeights) {
  // sets first and last frames; inherited from MotionModel
  init(tracks, nTracks, minSetSize());

  // estimate pairwise transforms first:
  MotionModel::fromTracks(tracks, nTracks, sqWeights);

  // then compute cumulative transforms
  tCumulative_.clear();
  tCumulative_.resize(t_.offset(), t_.size() + 1);
  tCumulative_[firstFrame()] = Matx33f::eye();
  for (int frame = firstFrame(); frame < lastFrame(); ++frame) {
    tCumulative_[frame + 1] = t_[frame] * tCumulative_[frame];
  }
}

Matx33f Homography::estimateTransform(
    const vector<Vec2f>& img1Obs,
    const vector<Vec2f>& img2Obs,
    const vector<float>& sqWeights) {
  // check for degeneracies - these happen in first frames where tracks
  // lie on a grid, and opencv chokes.
  bool collinear = true;
  for (int i = 0; i < img1Obs.size() - 3; ++i) {
    // check i, i+1, and i+2 for collinearity; if not collinear, go ahead
    // points a,b,c are collinear iff det |a-b, a-c| == 0
    Matx22f diffs1;
    hconcat(img1Obs[i] - img1Obs[i + 1], img1Obs[i] - img1Obs[i + 2], diffs1);

    Matx22f diffs2;
    hconcat(img2Obs[i] - img2Obs[i + 1], img2Obs[i] - img2Obs[i + 2], diffs2);

    if (determinant(diffs1) > 1e-8 && determinant(diffs2) > 1e-8) {
      collinear = false;
    }
  }
  if (collinear) {
    return Matx33f::eye();
  }
  if (sqWeights.size() == 0) {
    return findHomography(img1Obs, img2Obs);
  } else {
    return weightedFindHomography(img1Obs, img2Obs, sqWeights);
  }
}

cv::Matx33f Homography::estimateTransformRobust(const vector<Vec2f> &img1Obs,
                                                const vector<Vec2f> &img2Obs) {
  if (img1Obs.size() < minSetSize()) {
    return cv::Matx33f::eye();
  } else {
    return findHomography(img1Obs, img2Obs, cv::RANSAC, 6.0);
  }
}

float Homography::estimateConfidence(const vector<Vec2f> &img1Obs,
                                     const vector<Vec2f> &img2Obs,
                                     const Matx33f &transform) {
  return 1.0;
  //// compute singular values of H
  //Vec3d w;

  //SVD::compute(transform, w, SVD::NO_UV);
  //float condition = w[0] / w[2];
  ////if (condition > kHomographyConditionThresh) {
  //if (w[2] < 1e-15) {
    //return 0.2;
  //} else {
    //return 1.0;
  //}
}

Matx33f Homography::weightedFindHomography(
    const vector<Vec2f>& img1Obs,
    const vector<Vec2f>& img2Obs,
    const vector<float>& sqWeights) {
  assert(sqWeights.size() == img1Obs.size() && sqWeights.size() == img1Obs.size());

  const int n = img1Obs.size();

  vector<Vec2f> img1Norm;
  vector<Vec2f> img2Norm;
  Vec2f cent1(0, 0);
  Vec2f cent2(0, 0);
  Vec2f scale1(0, 0);
  Vec2f scale2(0, 0);

  normalizeCorrespondences(
      img1Obs, img2Obs, img1Norm, img2Norm, cent1, cent2, scale1, scale2);

  Matx33d T1(scale1(0), 0, -cent1(0) * scale1(0),
             0, scale1(1), -cent1(1) * scale1(1),
             0, 0, 1);
  Matx33d T2(1.0 / scale2(0), 0, cent2(0),
             0, 1.0 / scale2(1), cent2(1),
             0, 0, 1);

  // form upper triangular portion of A'A
  double AtA[9][9];
  Mat matAtA(9, 9, CV_64FC1, &AtA[0][0]);
  matAtA.setTo(0);
  for (int i = 0; i < n; ++i) {
    float x1 = img1Norm[i](0);
    float y1 = img1Norm[i](1);
    float x2 = img2Norm[i](0);
    float y2 = img2Norm[i](1);
    //float w2 = pow(weights[i], 2);
    float w2 = sqWeights[i];

    float r1[] = {-x1, -y1, -1, 0, 0, 0, x2 * x1, x2 * y1, x2};
    float r2[] = {0, 0, 0, -x1, -y1, -1, y2 * x1, y2 * y1, y2};
    for (int j = 0; j < 9; ++j) {
      for (int k = j; k < 9; ++k) {
        AtA[j][k] += w2 * r1[j] * r1[k] + w2 * r2[j] * r2[k];
      }
    }
  }
  // fill in symmetric AtA
  completeSymm(matAtA);

  // compute nullspace (smallest eigenvector of AtA)
  Mat evals;
  double evecs[9][9];
  Mat matEvecs(9, 9, CV_64FC1, &evecs[0][0]);
  eigen(matAtA, evals, matEvecs);

  // build the full transform including normalization
  Matx33d resultDouble(evecs[8]);
  Matx33d resultNorm = T2 * (resultDouble * T1);

  // divide by lower right entry to normalize H
  Matx33f result;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result(i, j) = (float)(resultNorm(i, j) / resultNorm(2, 2));
    }
  }
  return result;
}

