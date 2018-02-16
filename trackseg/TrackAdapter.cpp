#include <fstream>
#include <iostream>

#include "TrackRansac.h"
#include "../common/TrackTable.h"

using namespace std;
using namespace cv;

int reverse(int nFrames, int frameNum) {
  return nFrames - 1 - frameNum;
}
template <typename TrackExtraType>
void brox2tracktable(TrackTable<Observation2d, TrackExtraType>& table, string broxfile, bool backwards) {
  // read ascii track data from broxfile created by Brox'sdensetrack code
  // into a TrackTable and serialize it to outputfile.
  int nFrames;
  int nTracks;
  int trackId;
  int trackLength;
  int frameNum;

  ifstream infile(broxfile);
  infile >> nFrames;
  infile >> nTracks;

  // make sure table has enough frames
  while (table.numFrames() < nFrames) {
    table.addFrame();
  }

  for (int i = 0; i < nTracks; ++i) {
    // note that the Brox tracking code just uses 0 tid for all tracks
    infile >> trackId >> trackLength;

    vector<int> fNums(trackLength);
    vector<Observation2d> obs(trackLength);

    // add the rest of the observations
    for (int j = 0; j < trackLength; ++j) {
        infile >> obs[j].loc(0) >> obs[j].loc(1) >> frameNum;
        if (backwards) {
          frameNum = reverse(nFrames, frameNum);
        }
        fNums[j] = frameNum;
    }

    if (backwards) {
      int tid = table.createTrack(fNums[trackLength-1], obs[trackLength-1]);
      for (int j = trackLength-2; j >= 0; --j) {
        table.addObs(tid, fNums[j], obs[j]);
      }
    } else {
      int tid = table.createTrack(fNums[0], obs[0]);
      for (int j = 1; j < trackLength; ++j) {
        table.addObs(tid, fNums[j], obs[j]);
      }
    }
    //for (int j = 0; j < ...
      //table.addObs(tid, frameNum, obs);
  }
}

template <typename TrackExtraType>
void tracktable2brox(string inputfile, string outputfile) {
  // output serialized tracktable to brox-format ascii file (for debugging)
  // this may break code that reads the brox file if any tracks were deleted,
  // because we write numTracks out at the beginning of the file
  FILE* fin = fopen(inputfile.c_str(), "r");
  TrackTable<Observation2d, TrackExtraType> tt;
  tt.deserialize(fin);

  ofstream outfile(outputfile);
  outfile << tt.numFrames() << endl;
  outfile << tt.numTracks() << endl;

  for (int tid = 0; tid < tt.numTracks(); ++tid) {
    if (tt.hasTrack(tid)) {
      const Track<Observation2d, TrackExtraType>& track = tt.track(tid);
      outfile << tid << " " << track.length() << endl;
      for (int fnum = track.firstFrame(); fnum <= track.lastFrame(); ++fnum) {
        const Observation2d ob = track.obs(fnum);
        outfile << ob.loc[0] << " " << ob.loc[1] << " " << fnum << endl;
      }
    }
  }
}

int main(int argc, char* argv[]) {
  CommandLineParser parser(
      argc,
      argv,
      "{help h | | print this message }"
      "{@informat  | brox   | input track format (default: brox)}"
      "{@outformat | tt     | output track format (default: tt)}"
      "{@inputfile |        | input track file }"
      "{@output    |        | output filename }"
      "{reverse r  |        | backwards input track file}");

  if (parser.has("help") || argc == 1) {
    parser.printMessage();
    return 0;
  }

  if (!parser.check()) {
    parser.printErrors();
    return 0;
  }

  string informat = parser.get<string>("@informat");
  string outformat = parser.get<string>("@outformat");
  if (!informat.compare(outformat)) {
    cout << "informat and outformat should be different" << endl;
    return 0;
  }
  string inputfile = parser.get<string>("@inputfile");
  string outputfile = parser.get<string>("@output");
  string revinput = parser.get<string>("reverse");

  if ((informat == "brox") && outformat == "tt") {
    // create table
    TrackTable<Observation2d, RansacData> table;
    table.init(0);
    // add forward tracks
    brox2tracktable<RansacData>(table, inputfile, false);

    //add reverse tracks if applicable
    if (revinput != "") {
      brox2tracktable<RansacData>(table, revinput, true);
    }
    cout << "Saving out with " << table.numTracks() << " tracks." << endl;
    FILE* fout = fopen(outputfile.c_str(), "w");
    table.serialize(fout);
    fclose(fout);

  } else if ((informat == "tt") && (outformat == "brox")) {
    cout << "ExtraType is RansacData" << endl;
    tracktable2brox<RansacData>(inputfile, outputfile);
  } else {
    cout << "invalid  to/from formats; use tt or brox" << endl;
  }
  return 0;
}

