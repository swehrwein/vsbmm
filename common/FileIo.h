#pragma once

#include <string>
#include <vector>
#include <set>

// Read a value from a file
template<typename T>
T fread(FILE * fin) {
  T temp;
  fread(&temp, sizeof(T), 1, fin);
  return temp;
}

// Write a value to a file
template<typename T>
void fwrite(FILE * fout, const T & value) {
  fwrite(&value, sizeof(T), 1, fout);
}

// Write contents of a vector to a file
template<typename T>
void fwritev(FILE * fout, const std::vector<T> & vec) {
  size_t len = vec.size();
  fwrite(fout, len);
  if (len > 0) {
    fwrite(&vec[0], sizeof(T), len, fout);
  }
}

// Write a sequence to a file, e.g., fwriteseq(fout, set.begin(), set.size()).
template<typename Iter>
void fwriteseq(FILE * fout, Iter it, const size_t size) {
  fwrite(fout, size);
  for (size_t i = 0; i < size; ++i) {
    fwrite(fout, *it++);
  }
}

// Read a sequence from a file, e.g., freadseq(fout, std::back_inserter(vec)).
template<typename Iter>
void freadseq(FILE * fin, Iter it) {
  size_t size = fread<size_t>(fin);
  for (size_t i = 0; i < size; ++i) {
    *it = fread<typename Iter::container_type::value_type>(fin);
    it++;
  }
}

template<typename T>
void fwriteseq(FILE * fout, const std::set<T> & set) {
  fwriteseq(fout, set.begin(), set.size());
}

template<typename T>
void freadseq(FILE * fin, std::set<T> & set) {
  set.clear();
  freadseq(fin, std::inserter(set, set.begin()));
}

