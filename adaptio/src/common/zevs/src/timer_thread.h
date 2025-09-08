#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>

namespace zevs {

class TimerSubscriber {
 public:
  virtual ~TimerSubscriber()          = default;
  virtual void Init()                 = 0;
  virtual void Send(uint32_t task_id) = 0;
};

using TimerSubscriberPtr = std::unique_ptr<TimerSubscriber>;

using SteadyClock      = std::chrono::steady_clock;
using DueTime          = std::chrono::time_point<SteadyClock>;
using DurationMilliSec = std::chrono::milliseconds;

class Timeout {
 public:
  Timeout(uint32_t task_id, DueTime due);
  auto TaskId() const -> uint32_t;  // NOLINT(*-use-nodiscard)
  auto Due() const -> DueTime;      // NOLINT(*-use-nodiscard)
 private:
  uint32_t task_id_;
  DueTime due_;
};

class Task {
 public:
  Task(uint32_t task_id, DurationMilliSec duration, bool periodic);
  auto TaskId() const -> uint32_t;            // NOLINT(*-use-nodiscard)
  auto Duration() const -> DurationMilliSec;  // NOLINT(*-use-nodiscard)
  auto Periodic() const -> bool;              // NOLINT(*-use-nodiscard)
 private:
  uint32_t task_id_;
  DurationMilliSec duration_;
  bool periodic_;
};

class TimerThread {
 public:
  explicit TimerThread(TimerSubscriberPtr subscriber);

  auto Request(const DurationMilliSec& duration, bool periodic) -> uint32_t;
  void Cancel(uint32_t task_id);
  void Run();
  void Exit();

 private:
  TimerSubscriberPtr subscriber_;

  std::condition_variable condition_var_;
  std::mutex mutex_;
  bool exit_ = false;

  std::multiset<Timeout> timeouts_;
  std::unordered_map<uint32_t, Task> tasks_;
  uint32_t task_id_count_ = 0;
};

}  // namespace zevs
