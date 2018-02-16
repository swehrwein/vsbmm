// Include the track table inl file bc the compiler need to know the
// implementation to instantiate the template classes.

#include "TrackRansac.h"

#include "../common/TrackTable.h"
#include "../common/TrackTable-inl.h"

// Explicit template instantiation
template class Track<Observation2d>;
template class TrackTable<Observation2d>;

template class Track<Observation2d, RansacData>;
template class TrackTable<Observation2d, RansacData>;

