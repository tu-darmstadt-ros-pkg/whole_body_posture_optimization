#ifndef WHOLE_BODY_POSTURE_OPTIMIZATION_WORKER_THREAD_H
#define WHOLE_BODY_POSTURE_OPTIMIZATION_WORKER_THREAD_H

#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>

namespace whole_body_posture_optimization {

class WorkerThread {
public:
  WorkerThread();
  ~WorkerThread();

  void addTask(const std::function<void()>& task);
private:
  void run();

  std::thread worker_;
  std::queue<std::function<void()>> task_queue_;
  std::mutex mtx_;
  std::condition_variable cond_var_;
  std::atomic<bool> stop_;
};

}  // namespace whole_body_posture_optimization

#endif  // WHOLE_BODY_POSTURE_OPTIMIZATION_WORKER_THREAD_H
