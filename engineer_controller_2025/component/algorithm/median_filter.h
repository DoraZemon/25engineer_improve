/**
  ******************************************************************************
  * @file           : median_filter.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 2025/5/5
  ******************************************************************************
  */


#ifndef AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_MEDIAN_FILTER_H_
#define AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_MEDIAN_FILTER_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++

#include <vector>
#include <deque>
#include <algorithm>
#include <stdexcept>
#include <iostream>

class MedianFilter {
 private:
  const int windowSize;    // 必须为奇数
  std::deque<float> data; // 使用双端队列维护数据窗口

  // 快速插入排序辅助函数
  void insertSorted(std::vector<float> &sortedWindow, float value) {
      sortedWindow.insert(std::upper_bound(sortedWindow.begin(),
                                           sortedWindow.end(), value), value);
  }

 public:
  // 构造函数：初始化窗口大小
  MedianFilter(int size) : windowSize(size) {
  }

  // 添加数据并获取当前滤波结果
  float addData(float newValue) {
      // 维护数据窗口
      data.push_back(newValue);
      if (data.size() > windowSize) {
          data.pop_front();
      }

      // 创建排序副本
      std::vector<float> sorted(data.begin(), data.end());
      std::sort(sorted.begin(), sorted.end());

      // 计算中位数
      const int size = sorted.size();
      if (size % 2 == 1) {
          return sorted[size / 2];
      } else {
          return (sorted[size / 2 - 1] + sorted[size / 2]) / 2.0;
      }
  }

  // 获取当前有效数据量
  int validSize() const { return data.size(); }
};


#endif //AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_MEDIAN_FILTER_H_
