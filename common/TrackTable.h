// Classes for storing tracks and observations in a track table.
//
// An observation is a point location in a frame. It is a templated type, two
// example implementations are provided:
// * Observation2D: used in the "regular video" tracker. Stores the 2D location
//   of the point in the frame
// * Observation3dCube: used in the 360 video tracker. Stores two redundant
//   representation of a point: 3D unit vector and cube face + 2D location.
//
// A track is a collection of observations in consecutive frames. The track
// class has a templated variable data that can store any user-provided
// per-track data.
#pragma once

#include <deque>
#include <set>
#include <memory>

#include <opencv2/core/core.hpp>

#include "OffsetVector.h"

// Observation structure for 2D video tracking
struct Observation2d {
  cv::Vec2f loc;

  Observation2d() = default;

  Observation2d(cv::Vec2f loc) :
      loc(loc) {
  }
};

// Observation structure for 360 video tracking. It contains a redundant
// representation of the observation: (1) a unit 3D vector, (2) a cube face
// index + 2D location within the cube face.
struct Observation3dCube {
  cv::Vec3f dir;
  unsigned char face;
  cv::Vec2f loc;

  Observation3dCube() = default;

  Observation3dCube(cv::Vec3f dir, unsigned char face, cv::Vec2f loc) :
      dir(dir), face(face), loc(loc) {
  }
};

// Dummy track extra data type that has no storage and does nothing
struct TrackExtraTypeNothing {
  void serialize(FILE * fout) const {
  }

  void deserialize(FILE * fin) {
  }
};

// Forward declaration of class TrackTable
template <typename ObsType, typename TrackExtraType> class TrackTable;

// class Track
template <typename ObsType, typename TrackExtraType = TrackExtraTypeNothing>
class Track {
  // Allow TrackTable to mess with internal structures of Track
  friend class TrackTable<ObsType, TrackExtraType>;

public:
  // Constructor
  Track() = default;

  // Disable copy and assignment
  Track(const Track &) = delete;
  Track & operator=(const Track &) = delete;

  // Access to observations
  const ObsType & obs(const int frame) const;
  ObsType & obs(const int frame);

  // Is the track visible in a frame?
  bool inFrame(const int frame) const;

  // First / last frames the track is observed in.
  int firstFrame() const;
  int lastFrame() const;

  // Length of the track
  int length() const;

  // Save / load track from a file.
  void serialize(FILE * fout) const;
  void deserialize(FILE * fin);

  // Templated user-provided extra per-track data
  TrackExtraType data;

private:
  OffsetVector<ObsType> obs_;
};

// class TrackTable
template <typename ObsType, typename TrackExtraType = TrackExtraTypeNothing>
class TrackTable {
public:
  // Types
  typedef Track<ObsType, TrackExtraType> TrackType;
  typedef std::shared_ptr<TrackType> PTrackType;
  typedef std::set<int> FrameTracks;

  // Constructors
  TrackTable() = default;
  TrackTable(const int startFrame);

  // Disable copy and assignment
  TrackTable(const TrackTable &) = delete;
  TrackTable & operator=(const TrackTable &) = delete;

  // Reinitialize
  void init(const int startFrame);

  // Adds a new tracked frame. This has to be done before adding any
  // observations in this frame.
  void addFrame();

  // Return the start frame, last tracked frame, number of frames
  int startFrame() const;
  int lastFrame() const;
  int numFrames() const;

  // Is the frame index valid?
  bool hasFrame(const int frame) const;

  // Clear everything
  void clear();

  // Clear tracks in a frame
  void clearFrameTracks(const int frame);

  // Creates a new track, returns its ID. IDs are unique and assigned in a
  // running fashion.
  size_t createTrack(const int frame, ObsType & obs);

  // Delete a track
  void deleteTrack(const int trackId);

  // Make a track shorter
  void shortenTrack(const int trackId, const int lastFrame);

  // Return number of tracks. Some IDs might be "invalid" because tracks have
  // been deleted.
  size_t numTracks() const;

  // Check whether a track ID is valid. It can be invalid either because the ID
  // is out of bounds, or the track has been deleted.
  bool hasTrack(const size_t id);

  // Return a reference or const reference to a track
  const TrackType & track(const size_t id) const;
  TrackType & track(const size_t id);

  // Return a shared_ptr to a track
  std::shared_ptr<Track<ObsType, TrackExtraType>> trackPtr(
      const size_t id) const;

  // Add an observation to a track
  void addObs(const int trackId, const int frame, const ObsType & obs);

  // Remove an observation from a track
  void removeObs(const int trackId, const int frame);

  // Returns a sequence of track IDs observed in a frame
  const FrameTracks & frameTracks(const int frame) const;

  // Returns the number of tracks observed in a frame
  int frameTracksCount(const int frame) const;

  // Save / load track table from a file.
  void serialize(FILE * fout);
  void deserialize(FILE * fin);

  // Checks internal consistency of the track table
  bool checkSanity();

private:
  std::deque<PTrackType> tracks_;
  OffsetVector<std::shared_ptr<std::set<int>>> frames_;
};

