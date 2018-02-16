#include "TrackVis.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

void blendBuffer(Mat3b& frame, Mat4b overlayBGRA, bool flipAxes) {
  assert(frame.rows == overlayBGRA.rows && frame.cols == overlayBGRA.cols);
  for (int r = 0; r < frame.rows; ++r) {
    Vec3b* frameRowPtr = frame.ptr<Vec3b>(r);
    Vec4b* overlayRowPtr = overlayBGRA.ptr<Vec4b>(r);
    for (int c = 0; c < frame.cols; ++c) {
      float alpha = 1.0 / 255 * overlayRowPtr[c](3);
      Vec4f oVal(overlayRowPtr[c]);
      Vec3f oColor(oVal(0), oVal(1), oVal(2));
      frameRowPtr[c] = Vec3b((1 - alpha) * Vec3f(frameRowPtr[c]) + alpha * oColor);
    }
  }
}

void draw(
    Mat3b& frame,
    const vector<Vec2f>& actual,
    const vector<pair<Vec2f, bool>>& predicted,
    const vector<float>& colors,
    const int cmap,
    const int maxLineLength) {
  assert(colors.size() == 0 || colors.size() == actual.size());
  float colorClip = 1.0f;

  Mat3b overlay = Mat3b::zeros(frame.rows, frame.cols);
  Mat1b alpha = Mat1b::zeros(frame.rows, frame.cols);
  Scalar alphaLevel(250);

  for (int i = 0; i < actual.size(); ++i) {
    float color;
    if (colors.size() == 0) {
      // if no user specified colors, compute color based on distance
      if (predicted[i].second) { // if it has a value
        color = norm(actual[i], predicted[i].first);
      } else {
        color = colorClip;
      }
    } else {
      color = colors[i];
    }
    uint8_t c = (uint8_t)(255 * color / colorClip);
    Scalar level(c, c, c, 255);

    Vec2f pred;
    // clamp distance of line
    Vec2f diff = actual[i] - predicted[i].first;
    float dist = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
    if (!predicted[i].second) { // no prediction, draw no line
      pred = actual[i];
    } else if (dist > maxLineLength) { // line is too long, clamp it
      diff *= (1.0 / dist);
      pred = actual[i] + maxLineLength*diff;
    } else {
      pred = predicted[i].first;
    }

    // subpixel drawing messes up colors, so just round for now
    Point oneone(1,1);
    Point pt(round(actual[i](0)), round(actual[i](1)));
    rectangle(overlay, pt - oneone, pt + oneone, level, -1);
    rectangle(alpha, pt-oneone, pt+oneone, alphaLevel, -1);

    if (predicted[i].second) {
      Point2i actPt(actual[i]);
      Point2i predPt(pred);
      line(overlay, actPt, predPt, level, 1, CV_AA);
      line(alpha, actPt, predPt, alphaLevel, 1, CV_AA);
    }
  }
  Mat3b overlayMapped;
  applyColorMap(overlay, overlayMapped, cmap);

  Mat4b overlayBGRA(overlay.rows, overlay.cols);
  Mat src[] = {overlayMapped, alpha};
  int fromTo[] = {0, 0, 1, 1, 2, 2, 3, 3};
  mixChannels(src, 2, &overlayBGRA, 1, fromTo, 4);

  blendBuffer(frame, overlayBGRA, false); // draw overlay onto frame
}

