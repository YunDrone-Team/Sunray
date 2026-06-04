#ifndef LATESTDATAHOLDER_H
#define LATESTDATAHOLDER_H


#include <mutex>
#include <utility>

// 线程安全的最新数据持有器：仅保存最新一条数据，取出即清空（纯非阻塞，无等待）
template <typename T>
class LatestDataHolder {
public:
    LatestDataHolder() : hasData(false) {}

    // 禁止拷贝和移动（避免多线程下的资源竞争）
    LatestDataHolder(const LatestDataHolder&) = delete;
    LatestDataHolder& operator=(const LatestDataHolder&) = delete;
    LatestDataHolder(LatestDataHolder&&) = delete;
    LatestDataHolder& operator=(LatestDataHolder&&) = delete;

    /**
     * @brief 写入数据（覆盖旧数据，仅保留最新）
     * @param data 要写入的最新数据
     */
    void push(const T& data);

    /**
     * @brief 非阻塞取出最新数据
     * @param data 输出参数：取出的最新数据（仅当返回true时有效）
     * @return true-取出成功（有数据），false-取出失败（无数据）
     * @note 数据只取一次，取出后重置标志位，等待下次push会重新写入
     */
    bool pop(T& data);

    /**
     * @brief 清空当前存储的最新数据
     */
    void clear();

    /**
     * @brief 检查是否有未取出的最新数据
     * @return true-有数据，false-无数据
     */
    bool hasDataAvailable() const;

private:
    mutable std::mutex mtx;  // mutable允许const方法加锁
    bool hasData;            // 是否有未取出的最新数据
    T latestData;            // 存储的最新数据（仅当hasData=true时有效）
};

// 模板类实现（必须放在头文件）
template <typename T>
void LatestDataHolder<T>::push(const T& data) {
    std::lock_guard<std::mutex> lock(mtx);
    // 覆盖旧数据，标记有新数据
    latestData = data;
    hasData = true;
}

template <typename T>
bool LatestDataHolder<T>::pop(T& data) {
    std::lock_guard<std::mutex> lock(mtx);
    // 无数据 → 直接返回失败
    if (!hasData) {
        return false;
    }
    // 有数据 → 取出并清空
    data = latestData;
    hasData = false; // 取出后立即清空，下次pop返回失败（直到新push）
    return true;
}

template <typename T>
void LatestDataHolder<T>::clear() {
    std::lock_guard<std::mutex> lock(mtx);
    hasData = false; // 标记无数据，下次push会覆盖
}

template <typename T>
bool LatestDataHolder<T>::hasDataAvailable() const {
    std::lock_guard<std::mutex> lock(mtx);
    return hasData;
}

#endif // LATESTDATAHOLDER_H
