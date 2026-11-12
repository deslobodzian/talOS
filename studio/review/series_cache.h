#pragma once

// Decoded series kept for the next request of the same range: an LRU whose
// accounted bytes (each entry's buckets, its key and a fixed per-entry
// overhead) never exceed its capacity (--cache-mb). An entry larger than the
// whole capacity is not kept.

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "studio/review/session.h"

namespace studio::review {

class SeriesCache {
 public:
  // The list node, the map node and the shared Series block, rounded up.
  static constexpr std::size_t kEntryOverheadBytes = 256;

  explicit SeriesCache(std::size_t capacity_bytes)
      : capacity_(capacity_bytes) {}

  // The entry under `key`, now the most recently used; null if none.
  std::shared_ptr<const Series> Get(const std::string& key) {
    const auto it = index_.find(key);
    if (it == index_.end()) return nullptr;
    entries_.splice(entries_.begin(), entries_, it->second);
    return it->second->series;
  }

  void Put(const std::string& key, std::shared_ptr<const Series> series) {
    const std::size_t bytes =
        series->bytes() + 2 * key.size() + kEntryOverheadBytes;
    if (const auto it = index_.find(key); it != index_.end()) {
      bytes_ -= it->second->bytes;
      entries_.erase(it->second);
      index_.erase(it);
    }
    if (bytes > capacity_) return;
    while (bytes_ + bytes > capacity_) {
      const Entry& oldest = entries_.back();
      bytes_ -= oldest.bytes;
      index_.erase(oldest.key);
      entries_.pop_back();
    }
    entries_.push_front({key, std::move(series), bytes});
    index_.emplace(key, entries_.begin());
    bytes_ += bytes;
  }

  std::size_t bytes() const { return bytes_; }
  std::size_t capacity() const { return capacity_; }
  std::size_t size() const { return entries_.size(); }

 private:
  struct Entry {
    std::string key;
    std::shared_ptr<const Series> series;
    std::size_t bytes;
  };
  std::size_t capacity_;
  std::size_t bytes_{0};
  std::list<Entry> entries_;
  std::unordered_map<std::string, std::list<Entry>::iterator> index_;
};

}  // namespace studio::review
