#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_WORKER_THREAD_POOL_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_WORKER_THREAD_POOL_H

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <vector>


namespace whole_body_posture_optimization {

class WorkerThreadPool {
public:
  explicit WorkerThreadPool(size_t num_threads) : stop_flag_(false) {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back(&WorkerThreadPool::workerThreadFunc, this);
    }
  }

  ~WorkerThreadPool() {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      stop_flag_ = true;
      task_queue = {};
    }
    cond_var_.notify_all();
    for (auto& worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  void addTask(std::function<void()> task) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      task_queue.push(std::move(task));
    }
    cond_var_.notify_one();
  }

private:
  void workerThreadFunc() {
    while (true) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] {
          return !task_queue.empty() || stop_flag_;
        });

        if (stop_flag_ && task_queue.empty()) {
          return;
        }

        task = std::move(task_queue.front());
        task_queue.pop();
      }
      task();
    }
  }

  std::vector<std::thread> workers;
  std::queue<std::function<void()>> task_queue;
  std::mutex mutex_;
  std::condition_variable cond_var_;
  std::atomic<bool> stop_flag_;
};

}  // namespace whole_body_posture_optimization

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_WORKER_THREAD_POOL_H
