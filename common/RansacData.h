#pragma once
#include "OffsetVector.h"
#include "FileIo.h"

struct RansacData {
  // A struct to be used as TrackExtraType in a TrackTable
  bool inlier = false; // whether it's an inlier
  std::vector<float> inlWts; // inlier weight
  OffsetVector<float> frameError; // inlier distance/error for each observation

  // File i/o:
  void serialize(FILE* fout) const {
    fwrite<bool>(fout, inlier);

    // inlWts
    size_t size = inlWts.size();
    fwrite<size_t>(fout, size);
    fwrite(&inlWts[0], sizeof(float), size, fout);

    // frameError
    int off = frameError.offset();
    size = frameError.size();
    fwrite<int>(fout, off);
    fwrite<size_t>(fout, size);
    fwrite(&frameError[off], sizeof(float), size, fout);
  }

  void deserialize(FILE* fin) {
    inlier = fread<bool>(fin);

    // inlWts
    size_t size = fread<size_t>(fin);
    inlWts.resize(size);
    fread(&inlWts[0], sizeof(float), size, fin);

    // frameError
    int off = fread<int>(fin);
    size = fread<size_t>(fin);
    frameError.resize(off, size);
    fread(&frameError[off], sizeof(float), size, fin);
  }
};

