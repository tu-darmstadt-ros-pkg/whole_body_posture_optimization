#include <whole_body_posture_optimization/util/worker_thread.h>

namespace whole_body_posture_optimization {

WorkerThread::WorkerThread() : stop_(false) {
  worker_ = std::thread(&WorkerThread::run, this);
}
WorkerThread::~WorkerThread() {
  // Signal to stop the worker thread
  {
    std::unique_lock<std::mutex> lock(mtx_);
    stop_ = true;
    task_queue_ = {};
  }
  cond_var_.notify_one();
  worker_.join();
}
void WorkerThread::addTask(const std::function<void()> &task) {
  std::unique_lock<std::mutex> lock(mtx_);
  task_queue_.push(task);
  cond_var_.notify_one();
}
void WorkerThread::run() {
  while (true) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cond_var_.wait(lock, [this] {
        return !task_queue_.empty() || stop_;
      });

      if (stop_ && task_queue_.empty()) {
        return;
      }

      task = task_queue_.front();
      task_queue_.pop();
    }
    task();
  }
}

}  // namespace whole_body_posture_optimization