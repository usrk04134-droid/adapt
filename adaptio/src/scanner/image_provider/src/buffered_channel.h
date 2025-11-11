#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <queue>

namespace scanner::image_provider {

template <typename Item>
class BufferedChannel {
 public:
  class Reader {
   public:
    explicit Reader(BufferedChannel* channel) : channel_(channel) {}

    auto Read() -> std::optional<Item> { return channel_->Read(); }

    auto IsEmpty() -> bool { return channel_->IsEmpty(); }

    auto Clone() -> std::unique_ptr<Reader> { return std::make_unique<Reader>(channel_); }

   private:
    BufferedChannel* channel_;
  };

  class Writer {
   public:
    explicit Writer(BufferedChannel* channel) : channel_(channel) {}

    void Write(Item item) { return channel_->Write(std::move(item)); }

    auto Clone() -> std::unique_ptr<Writer> { return std::make_unique<Writer>(channel_); }

   private:
    BufferedChannel* channel_;
  };

  using ReaderPtr = std::unique_ptr<typename BufferedChannel<Item>::Reader>;
  using WriterPtr = std::unique_ptr<typename BufferedChannel<Item>::Writer>;

  BufferedChannel() : queue_(std::make_unique<std::queue<Item>>()) {}
  BufferedChannel(BufferedChannel&)                     = delete;
  auto operator=(BufferedChannel&) -> BufferedChannel&  = delete;
  BufferedChannel(BufferedChannel&&)                    = delete;
  auto operator=(BufferedChannel&&) -> BufferedChannel& = delete;

  ~BufferedChannel() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!queue_->empty()) {
      queue_->pop();
    }
  };

  auto GetReader() -> std::unique_ptr<typename BufferedChannel<Item>::Reader> {
    return std::unique_ptr<BufferedChannel<Item>::Reader>(new BufferedChannel<Item>::Reader(this));
  }

  auto GetWriter() -> std::unique_ptr<typename BufferedChannel<Item>::Writer> {
    return std::unique_ptr<BufferedChannel<Item>::Writer>(new BufferedChannel<Item>::Writer(this));
  }

  friend Reader;
  friend Writer;

 private:
  std::mutex mutex_;
  std::unique_ptr<std::queue<Item>> queue_;

  auto Read() -> std::optional<Item> {
    std::lock_guard<std::mutex> lock(mutex_);

    if (queue_->empty()) {
      return std::nullopt;
    }

    Item item = std::move(queue_->front());
    queue_->pop();
    return item;
  }

  void Write(Item item) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_->push(std::move(item));
  }

  auto IsEmpty() -> bool {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_->empty();
  }
};

}  // namespace scanner::image_provider
