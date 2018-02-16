#pragma once

#include <opencv2/core/core.hpp>
#include <iostream>

#define HASH_SCALE 0x5bd1e995

#define STLHASH 0

#if STLHASH
#include <unordered_map>
#else
#include <dense_hash_map>
#endif


// hash function for a sequence of values
template <typename T, int ND>
struct SeqHash {
  std::size_t operator()(const T v) const {
    std::size_t result = 1;
    for (int i = 0; i < ND; ++i) {
      result = HASH_SCALE * result + v[i];
    }
    return result;
  }
};

template <typename T, int ND> struct IndEq {
  bool operator()(const T& s1, const T& s2) const {
    return s1 == s2;
  }
};

template <typename Val, int ND>
class Grid {
public:
  // index types
  typedef cv::Vec<int, ND> Ind; // integer
  typedef cv::Vec<float, ND> Indf; // float

  enum interp_t {
    NEAREST = 0,
    ADJACENT = 1,
    LINEAR = 2
  };

  interp_t interp_;
  Ind shape_;
  Indf scales_;

  Grid() = delete;
  Grid(Ind dims, Indf scales, interp_t interp = NEAREST)
      : shape_(dims), interp_(interp), scales_(scales) {
#if STLHASH
#else
    Ind emptyKey;
    for (int i = 0; i < ND; ++i) {
      emptyKey[i] = -1;
    }
    data.set_empty_key(emptyKey);
#endif

    switch(interp_) {
      case NEAREST: NNBRS = 1; break;
      case ADJACENT: NNBRS = ND + 1; break;
      case LINEAR: NNBRS = pow(2, ND); break;
      default:
        std::cout <<  "Invalid interpolation type in Grid.h" << std::endl;
    }

    neighbors_.resize(NNBRS);
    weights_.resize(NNBRS);

  }

  // container properties
  auto begin() {
    return data.begin();
  }
  auto end() {
    return data.end();
  }
  auto size() {
    return data.size();
  }

  // accessor that performs an insertion if I is not in the container
  Val& operator[](Ind I) {
    return data[I];
  }

  // check whether grid index I has a value
  bool has(Ind I) {
    return data.count(I);
  }

  void nearestNeighbor(const Indf& ind) {
    for (int d = 0; d < ND; ++d) {
      neighbors_[0][d] = std::round(ind[d]);
    }
    weights_[0] = 1.0f;
  }

  void adjacentNeighbors(const Indf& ind) {
    float totalWt = 0;
    Ind ceilInd, floorInd, integerDim, nearestDir;
    Ind& nearest = neighbors_[0];
    for (int d = 0; d < ND; ++d) {
      ceilInd[d] = std::ceil(ind[d]);
      floorInd[d] = std::floor(ind[d]);
      integerDim[d] = ceilInd[d] == floorInd[d];
      nearest[d] = std::round(ind[d]);
      nearestDir[d] = nearest[d] == ceilInd[d]; // 1 if nearest was ceil
    }

    weights_[0] = 1.0; // accumulate the weight for the nearest neighbor
    for (int d = 0; d < ND; ++d) {
      weights_[0] *= 1.0 - std::abs(ind[d] - nearest[d]);

      Ind& nbr = neighbors_[d+1];
      float& nbrWt = weights_[d+1];

      // change only dimension d to the opposite way from the nearest neighbor
      nbr = nearest;
      nbr[d] = nearestDir[d] ? floorInd[d] : ceilInd[d];

      // calculate its weight
      nbrWt = 1.0;
      for (int d = 0; d < ND; ++d) {
        nbrWt *= 1.0 - std::abs(ind[d] - nbr[d]);
      }
      totalWt += nbrWt;
    }
    // renormalize weights so they sum to 1
    totalWt += weights_[0];
    for (int nbrI = 0; nbrI < NNBRS; ++nbrI) {
      weights_[nbrI] /= totalWt;
    }
  }

  // multilinear interpolation needs to handle integer-valued indices as a
  // special case. this handles that and fills neighbors with the indices and
  // weights corresponding to all neighbors with nonzero weight
  void linterpNeighbors(const Indf& ind) {
    // compute the ceil and floor of each index
    Ind ceilInd, floorInd;
    for (int d = 0; d < ND; ++d) {
      ceilInd[d] = std::ceil(ind[d]);
      floorInd[d] = std::floor(ind[d]);
    }

    // nbrI is an ND-bit integer, where each bit determines whether to floor or
    // ceil the corresponding dimension
    for (int nbrI = 0; nbrI < NNBRS; ++nbrI) {
      Ind& nbr = neighbors_[nbrI];
      float& weight = weights_[nbrI];
      weight = 1.0;
      for (int d = 0; d < ND; ++d) {
        if (nbrI & (1 << d)) {
          nbr[d] = ceilInd[d];
        } else {
          nbr[d] = floorInd[d];
        }
        // weight = product over all dimensions of (1 - distance to nbr)
        weight *= (1.0 - std::abs(ind[d] - nbr[d]));
      }
    }
  }

  inline void getNeighbors(const Indf ind) {
    switch(interp_) {
      case NEAREST: nearestNeighbor(ind); break;
      case ADJACENT: adjacentNeighbors(ind); break;
      case LINEAR: linterpNeighbors(ind); break;
      default:
        std::cout <<  "Invalid interpolation type in Grid.h" << std::endl;
    }
  }

  inline void applyScale(Indf& ind) const {
    for (int d = 0; d < ND; ++d) {
      ind[d] *= scales_[d];
      ind[d] += 0.0001; // add epsilon to avoid integer dimensions
    }
  }

  // splat value to neighbors of ind via the chosen interpolation scheme
  void splat(Indf ind, const Val val) {
    //std::cout << ind << " ";
    applyScale(ind);
    //std::cout << ind << std::endl;
    getNeighbors(ind);

    // splat weighted value to each neighbor
    for (int nbrI = 0; nbrI < NNBRS; ++nbrI) {
      //std::cout << "     " << neighbors_[nbrI] << std::endl;
      data[neighbors_[nbrI]] += weights_[nbrI] * val;
    }
  }

  // interpolate a value at indf
  Val slice(Indf ind) {
    applyScale(ind);
    getNeighbors(ind);

    Val result;
    for (int nbrI = 0; nbrI < NNBRS; ++nbrI) {
      result += weights_[nbrI] * data[neighbors_[nbrI]];
    }
    return result;
  }

  // apply a 5-tap Gaussian blur kernel to the grid
  void blur() {
    MapType newMap;
#if STLHASH
#else
    Ind emptyKey;
    for (int i = 0; i < ND; ++i) {
      emptyKey[i] = -1;
    }
    newMap.set_empty_key(emptyKey);
#endif

    const float kernel[5] = {0.06136, 0.24477, 0.38774, 0.24477, 0.06136};

    for (int d = 0; d < ND; ++d) {
      for (typename MapType::const_iterator it = data.begin(); it != data.end();
           ++it) {
        Ind outInd = (*it).first;
        int dInd = outInd[d];
        Val inVal = (*it).second;
        for (int i = 0; i < 5; ++i) {
          outInd[d] = dInd - 2 + i;
          newMap[outInd] = kernel[i] * inVal;
        }
      }
    }
    data.swap(newMap);
  }

#if STLHASH
  typedef std::unordered_map<Ind, Val, SeqHash<Ind, ND>> MapType;
#else
  typedef google::dense_hash_map<Ind, Val, SeqHash<Ind, ND>, IndEq<Ind, ND>>
      MapType;
#endif
  MapType data;

  int NNBRS; // number of neighbors
  std::vector<Ind> neighbors_;
  std::vector<float> weights_;

};
