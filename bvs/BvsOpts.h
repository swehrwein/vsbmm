#pragma once

void readFloat(ifstream& infile, string& fieldName, float& value) {
  getline(infile, fieldName, ' '); // split on space
  string valString;
  getline(infile, valString); // split on newline
  value = std::stof(valString);
}
void readInt(ifstream& infile, string& fieldName, int& value) {
  getline(infile, fieldName, ' '); // split on space
  string valString;
  getline(infile, valString); // split on newline
  value = std::stoi(valString);
}
void readBool(ifstream& infile, string& fieldName, bool& value) {
  getline(infile, fieldName, ' '); // split on space
  string valString;
  getline(infile, valString); // split on newline
  value = (bool)std::stoi(valString);
}
void readIndf(ifstream& infile, string& fieldName, Indf& value) {
  getline(infile, fieldName, ' '); // split on space
  string valString;
  for (int i = 0; i < ND; ++i) {
    getline(infile, valString, ',');
    value(i) = std::stof(valString);
  }
  getline(infile, valString); // eat newline
}
void readInd(ifstream& infile, string& fieldName, Ind& value) {
  getline(infile, fieldName, ' '); // split on space
  string valString;
  for (int i = 0; i < ND; ++i) {
    getline(infile, valString, ',');
    value(i) = std::stoi(valString);
  }
  getline(infile, valString); // eat newline
}


struct BvsOpts {
  const int optsVersion = 0;
  float unaryWeight;
  float smoothWeight;

  // one of dimScales or gridSizes must be nonnegative for each dimension;
  // preference is given to dimScales.
  Indf dimScales; // scale for each dimension; n/a if negative
  Ind gridSizes; // size of grid for each dimension; n/a if negative

  // dimension weights when computing distance
  Indf dimWeights;

  GridT::interp_t interp;
  int gtFrames; // how many ground truth frames to give it
  bool dtDataTerm; // whether to use the time derivative data term
  float smoothReg; // smoothness regularization constant (min edge cost)
  int segThresh; // threshold on the sliced value

  // add fg cost radius away from tracks; disable using <= 0 value
  int noTracksRadius;
  int noTracksSpacing;
  float noTracksCost;
};


void readOptions(string filename, BvsOpts& p) {
  ifstream infile(filename);
  string fieldName;
  int fileVersion;
  readInt(infile, fieldName, fileVersion);
  cout << fieldName << " " << fileVersion << endl;

  if (fileVersion != p.optsVersion) {
    cout << "Mismatched optfile version." << endl;
    return;
  }

  readFloat(infile, fieldName, p.unaryWeight);
  cout << fieldName << " " << p.unaryWeight << endl;
  readFloat(infile, fieldName, p.smoothWeight);
  cout << fieldName << " " << p.smoothWeight << endl;

  readIndf(infile, fieldName, p.dimScales);
  cout << fieldName << " " << p.dimScales << endl;
  readInd(infile, fieldName, p.gridSizes);
  cout << fieldName << " " << p.gridSizes << endl;
  readIndf(infile, fieldName, p.dimWeights);
  cout << fieldName << " " << p.dimWeights << endl;

  int interp;
  readInt(infile, fieldName, interp);
  p.interp = (GridT::interp_t)interp;
  cout << fieldName << " " << interp << endl;

  readInt(infile, fieldName, p.gtFrames);
  cout << fieldName << " " << p.gtFrames << endl;
  readBool(infile, fieldName, p.dtDataTerm);
  cout << fieldName << " " << p.dtDataTerm << endl;
  readFloat(infile, fieldName, p.smoothReg);
  cout << fieldName << " " << p.smoothReg << endl;
  readInt(infile, fieldName, p.segThresh);
  cout << fieldName << " " << p.segThresh << endl;

  readInt(infile, fieldName, p.noTracksRadius);
  cout << fieldName << " " << p.noTracksRadius << endl;
  readInt(infile, fieldName, p.noTracksSpacing);
  cout << fieldName << " " << p.noTracksSpacing << endl;
  readFloat(infile, fieldName, p.noTracksCost);
  cout << fieldName << " " << p.noTracksCost << endl;
}


BvsOpts getProfile(string profName, BvsOpts& p) {

if (profName == "gtcal" || profName == "") {
  ////////////////
  // gtcal profile
  p.unaryWeight = 100;
  p.smoothWeight = 0.0;

  p.dimScales = Indf(1.0/10, 1.0/35, 1.0/35, 1.0/7.3, 1.0/8.5, 1.0/8.5);
  p.gridSizes = Ind(-1, -1, -1, -1, -1, -1);

  p.interp = GridT::ADJACENT;
  p.dimWeights = Indf(0.5, 0.5, 0.5, 1.3, 1.5, 1.5);
  p.gtFrames = -1;
  p.dtDataTerm = false;
  p.segThresh = 64;
  p.smoothReg = 0.01;

  p.noTracksRadius = 32;
  p.noTracksSpacing = 8;
  p.noTracksCost = 0.05;

} else if (profName == "bg") {
  ////////////////
  // BG profile
  p.unaryWeight = 100;
  p.smoothWeight = 0.001;

  p.dimScales = Indf(1.0/10, 1.0/35, 1.0/35, 1.0/7.3, 1.0/8.5, 1.0/8.5);
  p.gridSizes = Ind(-1, -1, -1, -1, -1, -1);

  p.interp = GridT::ADJACENT;
  p.dimWeights = Indf(0.5, 0.5, 0.5, 1.3, 1.5, 1.5);
  p.gtFrames = 0;
  p.dtDataTerm = false;
  p.segThresh = 64;
  p.smoothReg = 0.01;

  p.noTracksRadius = 32;
  p.noTracksSpacing = 8;
  p.noTracksCost = 0.05;
} else if (profName == "bgnn") {
  ////////////////
  // BGNN profile
  p.unaryWeight = 100;
  p.smoothWeight = .001;

  p.dimScales = Indf(1.0/10, 1.0/35, 1.0/35, 1.0/7.3, 1.0/8.5, 1.0/8.5);
  p.gridSizes = Ind(-1, -1, -1, -1, -1, -1);

  p.interp = GridT::NEAREST;
  p.dimWeights = Indf(0.5, 0.5, 0.5, 1.3, 1.5, 1.5);
  p.gtFrames = 0;
  p.dtDataTerm = false;
  p.smoothReg = 0.00;
  p.segThresh = 64;

  p.noTracksRadius = 32;
  p.noTracksSpacing = 8;
  p.noTracksCost = 0.05;
} else if (profName == "bvs") {
  //////////////
  // BVS profile
  p.unaryWeight = 100;
  p.smoothWeight = .001;

  p.dimScales = Indf(-1, 1.0/5, 1.0/5, -1, -1, -1);
  p.gridSizes = Ind(2, -1, -1, 35, 30, 30);

  p.interp = GridT::LINEAR;
  p.dimWeights = Indf(0.01, 0.5, 0.5, 1.3, 1.5, 1.5);
  p.gtFrames = 1;
  p.dtDataTerm = false;
  p.segThresh = 128;
  p.smoothReg = 0.00;
} else if (profName == "bvsfast") {
  //////////////
  // BVS fast
  p.unaryWeight = 100;
  p.smoothWeight = .001;

  p.dimScales = Indf(-1, 1.0/35, 1.0/35, -1, -1, -1);
  p.gridSizes = Ind(2, -1, -1, 35, 30, 30);

  p.interp = GridT::NEAREST;
  p.dimWeights = Indf(0.01, 0.5, 0.5, 1.3, 1.5, 1.5);
  p.gtFrames = 1;
  p.dtDataTerm = false;
  p.segThresh = 128;
  p.smoothReg = 0.00;
} else if (profName == "tlseg") {
  ////////////////
  // tlsegprofile
  p.unaryWeight = 1;
  p.smoothWeight = 0.001;

  p.dimScales = Indf(1.0/4, 1.0/35, 1.0/35, 1.0/10, 1.0/10, 1.0/10);
  p.gridSizes = Ind(-1, -1, -1, -1, -1, -1);

  p.interp = GridT::NEAREST;
  p.dimWeights = Indf(0.01, 0.5, 0.5, 1.3, 1.5, 1.5);
  p.gtFrames = 0;
  p.dtDataTerm = true;
  p.segThresh = 128;
  p.smoothReg = 0.01;
} else {
  cout << "invalid profile." << endl;
}
return p;

}
