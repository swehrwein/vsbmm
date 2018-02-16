#include "Grid.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

typedef Grid<Vec4f, 5> GridT;

void filter(string inputFn, string outputFn, float sXY, float sRGB) {

  Mat3b input = imread(inputFn);
  cout << input.rows << " " << input.cols << endl;
  Mat3b output(input.rows, input.cols);

  int gH = std::ceil(1.0 / sXY * input.rows);
  int gW = std::ceil(1.0 / sXY * input.cols);
  int gC = std::ceil(255.0 / sRGB);

  GridT::Ind shape(gH, gW, gC, gC, gC);
  GridT::Indf scales(sXY, sXY, sRGB, sRGB, sRGB);
  GridT g(shape, scales);
  GridT gBlur(shape, scales);

  // splat
  for (int r = 0; r < input.rows; ++r) {
    const Vec3b* rowPtr = input.ptr<Vec3b>(r);
    for (int c = 0; c < input.cols; ++c) {
      Vec2f xy(r, c);
      Vec3f rgb(rowPtr[c]);
      Vec<float, 5> lift(r/sXY, c/sXY, rgb(0)/sRGB, rgb(1)/sRGB, rgb(2)/sRGB);

      Vec4f value(rgb(0), rgb(1), rgb(2), 1.0f);
      g.splat(lift, value);
    }
  }

  // blur
  g.blur();

  // slice
  for (int r = 0; r < output.rows; ++r) {
    const Vec3b* inputRowPtr = input.ptr<Vec3b>(r);
    Vec3b* outputRowPtr = output.ptr<Vec3b>(r);
    for (int c = 0; c < output.cols; ++c) {
      Vec3f rgb(inputRowPtr[c]);
      Vec<float, 5> lift(r/sXY, c/sXY, rgb(0)/sRGB, rgb(1)/sRGB, rgb(2)/sRGB);
      Vec4f newColor = g.slice(lift);
      newColor /= newColor(3);
      outputRowPtr[c] = Vec3b(newColor(0), newColor(1), newColor(2));
    }
  }

  imwrite(outputFn, output);

}

int main(int argc, char* argv[]) {

  float sXY = 4.0;
  float sRGB = 10.0;

  if (argc < 3) {
    cout << "Usage: infile outfile" << endl;
    return 0;
  }

  filter(argv[1], argv[2], sXY, sRGB);
  return 0;

}
