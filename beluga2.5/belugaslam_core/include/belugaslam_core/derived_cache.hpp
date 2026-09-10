#ifndef BELUGASLAM_CORE_DERIVED_CACHE_HPP
#define BELUGASLAM_CORE_DERIVED_CACHE_HPP
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
namespace belugaslam {
// Shared by pose-only clones. Readers retain their payload even if a later
// eviction removes the cache's reference. Grids and graph state are never evicted.
template<class Payload>
class DerivedCache {
 public:
  template<class Builder>
  std::shared_ptr<const Payload> acquire(Builder&& builder) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!value_) value_ = std::make_shared<const Payload>(builder());
    last_use_ = ++clock_;
    return value_;
  }
  std::pair<std::size_t,std::uint64_t> statistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {value_ ? value_->bytes() : 0, last_use_};
  }
  void release() const {
    std::lock_guard<std::mutex> lock(mutex_);
    value_.reset();
  }
 private:
  mutable std::mutex mutex_;
  mutable std::shared_ptr<const Payload> value_;
  mutable std::uint64_t last_use_ = 0;
  inline static std::atomic<std::uint64_t> clock_{0};
};
}  // namespace belugaslam
#endif
