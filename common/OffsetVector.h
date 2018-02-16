#pragma once

template <typename T>
class OffsetVector {
public:
    OffsetVector() {}

    OffsetVector(const size_t offset, const size_t size) : offset_(offset) {
      allocate(size);
    }

    ~OffsetVector() {
      deallocate();
    }

    // Disable copying and assignment
    OffsetVector(const OffsetVector &) = delete;
    OffsetVector & operator=(const OffsetVector &) = delete;

    size_t offset() const {
      return offset_;
    }

    size_t size() const {
      return data_.size();
    }

    size_t firstIndex() const {
      return offset_;
    }

    size_t lastIndex() const {
      return offset_ + size() - 1;
    }

    bool empty() const {
      return data_.empty();
    }

    void clear() {
      data_.clear();
    }

    // Return true if the index can be accessed
    bool hasIndex(const size_t index) const {
      return (index >= offset_ && index < offset_ + size());
    }

    void resize(const size_t offset, const size_t size) {
      offset_ = offset;
      allocate(size);
    }

    typename std::vector<T>::const_iterator begin() const {
      return data_.begin();
    }

    typename std::vector<T>::iterator begin() {
      return data_.begin();
    }

    typename std::vector<T>::const_iterator end() const {
      return data_.end();
    }

    typename std::vector<T>::iterator end() {
      return data_.end();
    }

    const T & operator[](const size_t index) const {
      return offsetData_[index];
    }

    T & operator[](const size_t index) {
      return offsetData_[index];
    }

    const T & at(const size_t index) const {
      return offsetData_[index];
    }

    T & at(const size_t index) {
      return offsetData_[index];
    }

    // return reference to absolute indexed element
    const T & atAbs(const size_t index) const {
      return data_[index];
    }

    T & atAbs(const size_t index) {
      return data_[index];
    }

    void push_back(const T & el) {
      data_.push_back(el);

      // we need to update the offset address because raw data may have moved
      offsetData_ = data_.data() - offset_;
    }

private:
    void allocate(const size_t size) {
      if (size > 0) {
        data_.resize(size);
        offsetData_ = data_.data() - offset_;
      } else {
        data_.clear();
        offsetData_ = nullptr;
      }
    }

    void deallocate() {
      data_.clear();
      offsetData_ = nullptr;
    }

    std::vector<T> data_;
    T * offsetData_ = nullptr;
    size_t offset_ = 0;
};
