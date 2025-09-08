#include "timer_thread.h"

#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

namespace zevs {

inline auto operator<(const Timeout& lhs, const Timeout& rhs) -> bool { return lhs.Due() < rhs.Due(); }

Timeout::Timeout(uint32_t task_id, DueTime due) : task_id_(task_id), due_(due) {}

auto Timeout::TaskId() const -> uint32_t { return task_id_; }
auto Timeout::Due() const -> DueTime { return due_; }

Task::Task(uint32_t task_id, DurationMilliSec duration, bool periodic)
    : task_id_(task_id), duration_(duration), periodic_(periodic) {}

auto Task::TaskId() const -> uint32_t { return task_id_; }
auto Task::Duration() const -> DurationMilliSec { return duration_; }
auto Task::Periodic() const -> bool { return periodic_; }

// NOLINTNEXTLINE(*pass-by-value)
TimerThread::TimerThread(TimerSubscriberPtr subscriber) : subscriber_(std::move(subscriber)) {}

// Note 1: The lock is always released before calling notify_all which is customary
// and done for efficiency reasons.
//
// Note 2: The correct order to delete a TimerThread instance is:
// call Exit()
// join the thread
// call the destructor
//
// In a previous version the exit_ member was set to true in the destructor which caused
// an invalid read when the thread woke up (on notify_all) and read the exit_ member
// in the already deleted TimerThread object.
void TimerThread::Exit() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    exit_ = true;
  }
  condition_var_.notify_all();
}

auto TimerThread::Request(const DurationMilliSec& duration, bool periodic) -> uint32_t {
  uint32_t task_id = 0;

  {
    std::unique_lock<std::mutex> lock(mutex_);

    task_id = ++task_id_count_;
    Task task{task_id, duration, periodic};
    tasks_.insert({task_id, task});

    Timeout timeout{task_id, SteadyClock::now() + duration};
    timeouts_.insert(timeout);
  }

  condition_var_.notify_all();
  return task_id;
}

void TimerThread::Cancel(uint32_t task_id) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    tasks_.erase(task_id);
    std::erase_if(timeouts_, [task_id](auto const& timeout) { return timeout.TaskId() == task_id; });
  }

  condition_var_.notify_all();
}

void TimerThread::Run() {
  std::unique_lock<std::mutex> lock(mutex_);

  if (exit_) {
    // This can occur if TimerThread destructor is called before
    // lock is aquired in Run.
    return;
  }

  subscriber_->Init();  // creates the socket in this thread

  // Note that the mutex is owned/locked whenever the thread is not
  // waiting on the condition variable in the while loop below.
  while (!exit_) {
    if (!timeouts_.empty()) {
      auto task_id  = timeouts_.begin()->TaskId();
      auto due_time = timeouts_.begin()->Due();

      if (SteadyClock::now() >= due_time) {
        // Timer has expired, send timeout

        timeouts_.erase(timeouts_.begin());
        auto iter = tasks_.find(task_id);
        if (iter == tasks_.end()) {
          continue;
        }
        auto task = iter->second;

        subscriber_->Send(task_id);

        if (task.Periodic()) {
          // due is calculated from the old value rather than from now()
          // to avoid drift

          Timeout again_timeout{task_id, due_time + task.Duration()};
          timeouts_.insert(again_timeout);
        } else {
          tasks_.erase(task_id);
        }
      } else {
        condition_var_.wait_until(lock, due_time);
      }
    } else {
      // no requested timeouts
      condition_var_.wait(lock);
    }
  }
}

}  // namespace zevs
