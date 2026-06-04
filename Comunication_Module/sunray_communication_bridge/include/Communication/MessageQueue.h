#ifndef MESSAGEQUEUE_H
#define MESSAGEQUEUE_H


#include <queue>
#include <mutex>
#include <condition_variable>
#include <utility>

// 线程安全的生产者-消费者队列
template <typename T>
class MessageQueue {
public:
    MessageQueue() : isRunning(true) {}

    // 禁止拷贝和移动（避免多线程下的资源竞争）
    MessageQueue(const MessageQueue&) = delete;
    MessageQueue& operator=(const MessageQueue&) = delete;
    MessageQueue(MessageQueue&&) = delete;
    MessageQueue& operator=(MessageQueue&&) = delete;

    // 停止队列（通知消费者退出）
    void stop();

    // 入队（生产者调用）
    bool push(const T& data);

    // 出队（消费者调用，阻塞直到有数据/停止）
    bool pop(T& data);

    // 清空队列
    void clear();

    ~MessageQueue();

private:
    std::mutex mtx;
    std::condition_variable cv;
    std::queue<T> queue;
    bool isRunning;
};

// 模板类的实现必须放在头文件中（编译器需要实例化）
template <typename T>
void MessageQueue<T>::stop() {
    std::lock_guard<std::mutex> lock(mtx);
    isRunning = false;
    cv.notify_all();
}

template <typename T>
bool MessageQueue<T>::push(const T& data) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!isRunning) return false;
    queue.push(data);
    cv.notify_one();
    return true;
}

template <typename T>
bool MessageQueue<T>::pop(T& data) {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]() { return !isRunning || !queue.empty(); });

    if (!isRunning && queue.empty()) return false;
    data = queue.front();
    queue.pop();
    return true;
}

template <typename T>
void MessageQueue<T>::clear() {
    std::lock_guard<std::mutex> lock(mtx);
    std::queue<T> emptyQueue;
    std::swap(queue, emptyQueue);
}

template <typename T>
MessageQueue<T>::~MessageQueue() {
    stop();
}

#endif // MESSAGEQUEUE_H
