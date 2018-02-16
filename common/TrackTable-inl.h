#include "TrackTable.h"

#include "FileIo.h"

//
// Track implementation
//

// Const reference to observation
template <typename ObsType, typename ExtraType>
const ObsType & Track<ObsType, ExtraType>::obs(const int frame) const {
  return obs_[frame];
}

// Non-const reference to observation
template <typename ObsType, typename ExtraType>
ObsType & Track<ObsType, ExtraType>::obs(const int frame) {
  return obs_[frame];
}

// Is the track visible in a frame?
template <typename ObsType, typename ExtraType>
bool Track<ObsType, ExtraType>::inFrame(const int frame) const {
  return (frame >= firstFrame() && frame <= lastFrame());
}

// First frame the track is observed in.
template <typename ObsType, typename ExtraType>
int Track<ObsType, ExtraType>::firstFrame() const {
  return obs_.firstIndex();
}

// Last frame the track is observed in.
template <typename ObsType, typename ExtraType>
int Track<ObsType, ExtraType>::lastFrame() const {
  return obs_.lastIndex();
}

// Length of the track
template <typename ObsType, typename ExtraType>
int Track<ObsType, ExtraType>::length() const {
  return obs_.lastIndex() - obs_.firstIndex() + 1;
}

// Save track to a file.
template <typename ObsType, typename ExtraType>
void Track<ObsType, ExtraType>::serialize(FILE * fout) const {
  fwrite(fout, obs_.offset());
  size_t size = obs_.size();
  fwrite(fout, size);
  if (size > 0) {
    fwrite(&obs_.atAbs(0), sizeof(ObsType), size, fout);
  }
  data.serialize(fout);
}

// Load track from a file.
template <typename ObsType, typename ExtraType>
void Track<ObsType, ExtraType>::deserialize(FILE * fin) {
  size_t offset = fread<size_t>(fin);
  size_t size = fread<size_t>(fin);
  obs_.resize(offset, size);
  if (size > 0) {
    fread(&obs_.atAbs(0), sizeof(ObsType), size, fin);
  }
  data.deserialize(fin);
}

//
// TrackTable implementation
//

// Constructor
template <typename ObsType, typename TrackExtraType>
TrackTable<ObsType, TrackExtraType>::TrackTable(const int startFrame) {
  init(startFrame);
}

template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::init(const int startFrame) {
  frames_.resize(startFrame, 0);
  tracks_.clear();
}

// Adds a new tracked frame.
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::addFrame() {
  frames_.push_back(std::make_shared<std::set<int>>());
}

// Returns the start frame
template <typename ObsType, typename TrackExtraType>
int TrackTable<ObsType, TrackExtraType>::startFrame() const {
  return frames_.firstIndex();
}

// Returns the last tracked frame
template <typename ObsType, typename TrackExtraType>
int TrackTable<ObsType, TrackExtraType>::lastFrame() const {
  return frames_.lastIndex();
}

// Returns the number of frames
template <typename ObsType, typename TrackExtraType>
int TrackTable<ObsType, TrackExtraType>::numFrames() const {
  return frames_.size();
}

// Is the frame index valid?
template <typename ObsType, typename TrackExtraType>
bool TrackTable<ObsType, TrackExtraType>::hasFrame(const int frame) const {
  return frames_.hasIndex(frame);
}

// Clear everything
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::clear() {
  tracks_.clear();
  frames_.clear();
}

// Clear tracks in a frame
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::clearFrameTracks(const int frame) {
  const auto & frameTracks = *frames_[frame];
  for (auto it = frameTracks.begin(); it != frameTracks.end(); ++it) {
    assert(tracks_[*it] != nullptr);
    auto & t = *tracks_[*it];

    // Can only delete observations at the end of track!
    assert(frame == t.lastFrame());

    t.obs_.resize(t.obs_.offset(), t.obs_.size()-1);
  }
  frames_[frame]->clear();
}

// Creates a new track
template <typename ObsType, typename TrackExtraType>
size_t TrackTable<ObsType, TrackExtraType>::createTrack(
    const int frame, ObsType & obs) {
  // Create track struct
  auto pt = std::make_shared<TrackType>();
  pt->obs_.resize(frame, 1);
  pt->obs_[frame] = obs;

  // Add to tracks
  size_t trackId = tracks_.size();
  tracks_.push_back(pt);

  // Add to frame
  assert(frames_.hasIndex(frame));
  frames_[frame]->insert(trackId);

  return trackId;
}

// Delete a track
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::deleteTrack(const int trackId) {
  auto pt = tracks_[trackId];
  assert(pt != nullptr);

  // Remove from frames
  size_t lastIndex = pt->obs_.lastIndex();
  for (int frame = pt->obs_.firstIndex(); frame <= lastIndex; ++frame) {
    assert(frames_.hasIndex(frame));
    frames_[frame]->erase(trackId);
  }

  // Delete track
  tracks_[trackId] = nullptr;
}

// Make a track shorter
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::shortenTrack(
    const int trackId, const int lastFrame) {
  auto pt = tracks_[trackId];
  assert(pt != nullptr);
  assert(lastFrame >= pt->firstFrame());
  assert(lastFrame <= pt->lastFrame());
  assert(frames_.lastIndex() >= pt->lastFrame());

  for (int frame = lastFrame + 1; frame <= pt->lastFrame(); ++frame) {
    frames_[frame]->erase(trackId);
  }

  int firstFrame = pt->obs_.offset();
  pt->obs_.resize(firstFrame, lastFrame - firstFrame + 1);
}

// Return number of tracks.
template <typename ObsType, typename TrackExtraType>
size_t TrackTable<ObsType, TrackExtraType>::numTracks() const {
  return tracks_.size();
}

// Check whether a track ID is valid.
template <typename ObsType, typename TrackExtraType>
bool TrackTable<ObsType, TrackExtraType>::hasTrack(const size_t id) {
  if (id >= tracks_.size()) {
    return false;
  }
  return (tracks_[id] != nullptr);
}

// Returns a const reference to a track
template <typename ObsType, typename TrackExtraType>
const Track<ObsType, TrackExtraType> &
    TrackTable<ObsType, TrackExtraType>::track(const size_t id) const {
  assert(tracks_[id] != nullptr);
  return *tracks_[id];
}

// Returns a non-const reference to a track
template <typename ObsType, typename TrackExtraType>
Track<ObsType, TrackExtraType> &
  TrackTable<ObsType, TrackExtraType>::track(const size_t id) {
  assert(tracks_[id] != nullptr);
  return *tracks_[id];
}

// Returns a shared_ptr to a track
template <typename ObsType, typename TrackExtraType>
std::shared_ptr<Track<ObsType, TrackExtraType>>
TrackTable<ObsType, TrackExtraType>::trackPtr(const size_t id) const {
  return tracks_[id];
}

// Add an observation to a track
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::addObs(
    const int trackId, const int frame, const ObsType & obs) {
  auto pt = tracks_[trackId];
  assert(pt != nullptr);
  assert(frames_.hasIndex(frame));

  // Can only add observations at end of track
  assert(pt->lastFrame() == frame - 1);

  pt->obs_.push_back(obs);

  frames_[frame]->insert(trackId);
}

// Remove an observation from a track
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::removeObs(
    const int trackId, const int frame) {
  auto pt = tracks_[trackId];
  assert(pt != nullptr);

  // Can only remove observation at end of track
  assert(pt->lastFrame() == frame);
}

// Returns a sequence of track IDs observed in a frame
template <typename ObsType, typename TrackExtraType>
const std::set<int> &
    TrackTable<ObsType, TrackExtraType>::frameTracks(const int frame) const {
  assert(frames_.hasIndex(frame));
  return *frames_[frame];
}

// Returns the number of tracks observed in a frame
template <typename ObsType, typename TrackExtraType>
int TrackTable<ObsType, TrackExtraType>::frameTracksCount(
    const int frame) const {
  return frames_[frame]->size();
}

// Save track table to a file.
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::serialize(FILE * fout) {
  size_t numTracks = tracks_.size();
  fwrite(fout, numTracks);
  for (size_t trackId = 0; trackId < tracks_.size(); ++trackId) {
    auto pt = tracks_[trackId];
    if (pt == nullptr) {
      fwrite<bool>(fout, false);
    } else {
      fwrite<bool>(fout, true);
      pt->serialize(fout);
    }
  }

  fwrite(fout, frames_.offset());
  fwrite(fout, frames_.size());
  for (auto & frameTracks : frames_) {
    fwriteseq(fout, *frameTracks);
  }
}

// Load track table from a file.
template <typename ObsType, typename TrackExtraType>
void TrackTable<ObsType, TrackExtraType>::deserialize(FILE * fin) {
  size_t numTracks = fread<size_t>(fin);
  tracks_.resize(numTracks);

  for (size_t trackId = 0; trackId < numTracks; ++trackId) {
    bool valid = fread<bool>(fin);
    if (valid) {
      auto pt = std::make_shared<TrackType>();
      pt->deserialize(fin);
      tracks_[trackId] = pt;
    } else {
      tracks_[trackId] = nullptr;
    }
  }

  size_t offset = fread<size_t>(fin);
  size_t size = fread<size_t>(fin);
  frames_.resize(offset, size);
  for (auto & frameTracks : frames_) {
    frameTracks = std::make_shared<std::set<int>>();
    freadseq(fin, *frameTracks);
  }
}

// Checks internal consistency of the track table
template <typename ObsType, typename TrackExtraType>
bool TrackTable<ObsType, TrackExtraType>::checkSanity() {
  // Ensure that all tracks are reference in the frames sets
  for (size_t trackId = 0; trackId < tracks_.size(); ++trackId) {
    if (tracks_[trackId] == nullptr) {
      continue;
    }

    const auto & t = *tracks_[trackId];

    assert(t.length() >= 1);

    for (int frame = t.firstFrame(); frame < t.lastFrame(); ++frame) {
      assert(frames_[frame]->find(trackId) != frames_[frame]->end());
    }
  }

  // Ensure that all frames sets refer to actual tracks
  for (int frame = frames_.firstIndex();
      frame <= frames_.lastIndex(); ++frame) {
    for (const int trackId : *frames_[frame]) {
      assert(hasTrack(trackId));
      auto & t = track(trackId);
      assert(t.firstFrame() <= frame && frame <= t.lastFrame());
    }
  }

  return true;
}
