#pragma once
#include "AscomDriver.h"
#include <algorithm>
#include <deque>


// **ASCOM调焦器接口定义**
MIDL_INTERFACE("3FA0A3DB-8E5F-4E9C-BA3D-8A4C27CFF958")
IFocuser : public IAscomDriver {
public:
  virtual HRESULT STDMETHODCALLTYPE get_Absolute(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE Halt() = 0;
  virtual HRESULT STDMETHODCALLTYPE get_IsMoving(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Link(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_Link(VARIANT_BOOL newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_MaxIncrement(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_MaxStep(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE Move(long Position) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Position(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_StepSize(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_TempComp(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_TempComp(VARIANT_BOOL newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_TempCompAvailable(VARIANT_BOOL *
                                                          pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Temperature(double *pVal) = 0;
};

// **调焦器能力结构**
struct FocuserCapabilities {
  bool absolute_positioning = false;     // 绝对定位支持
  bool temperature_compensation = false; // 温度补偿
  bool temperature_available = false;    // 温度读取
  bool can_halt = false;                 // 停止运动

  long max_step = 0;      // 最大步数
  long max_increment = 0; // 最大单次移动步数
  double step_size = 0.0; // 步长（微米）

  bool loaded = false;
};

// **焦点测量数据**
struct FocusData {
  long position;      // 焦点位置
  double temperature; // 温度
  double fwhm;        // 半高全宽
  double hfd;         // 半通量直径
  std::chrono::system_clock::time_point timestamp;

  FocusData(long pos = 0, double temp = 0.0, double f = 0.0, double h = 0.0)
      : position(pos), temperature(temp), fwhm(f), hfd(h),
        timestamp(std::chrono::system_clock::now()) {}
};

// **自动调焦配置**
struct AutoFocusConfig {
  int search_range = 1000;          // 搜索范围（步数）
  int step_size = 50;               // 初始步长
  int fine_step_size = 10;          // 精细步长
  double target_fwhm = 2.0;         // 目标FWHM
  double tolerance = 0.1;           // 精度容差
  int max_iterations = 20;          // 最大迭代次数
  bool use_temperature_comp = true; // 使用温度补偿
  double temp_coefficient = -2.0;   // 温度系数（步数/摄氏度）
};

// **ASCOM调焦器驱动封装**
class AscomFocuser : public AscomDriverBase {
private:
  AscomPtr<IFocuser> focuser_;

  // **能力缓存**
  mutable FocuserCapabilities capabilities_;
  mutable std::mutex capabilities_mutex_;

  // **调焦历史管理**
  std::deque<FocusData> focus_history_;
  mutable std::mutex history_mutex_;
  static constexpr size_t MAX_HISTORY_SIZE = 1000;

  // **运动状态**
  std::atomic<bool> movement_abort_requested_{false};
  mutable std::mutex movement_mutex_;
  std::condition_variable movement_condition_;

  // **温度补偿**
  std::atomic<double> last_compensation_temp_{-273.15};
  std::atomic<long> compensation_reference_position_{0};

public:
  explicit AscomFocuser(const std::string &prog_id) : AscomDriverBase(prog_id) {
    focuser_ = AscomDeviceFactory::CreateDevice<IFocuser>(prog_id);
  }

  // **基础能力查询**
  const FocuserCapabilities &GetCapabilities() const {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);

    if (!capabilities_.loaded) {
      LoadCapabilities();
    }

    return capabilities_;
  }

  // **位置控制**
  long GetPosition() const {
    return MeasurePerformance("GetPosition", [this]() {
      long position;
      ThrowIfFailed(focuser_->get_Position(&position));
      return position;
    });
  }

  void MoveTo(long position) {
    auto caps = GetCapabilities();

    if (!caps.absolute_positioning) {
      throw AscomException(0x80040400, "Absolute positioning not supported");
    }

    if (position < 0 || position > caps.max_step) {
      throw AscomException(
          0x80040005, "Position out of range: " + std::to_string(position) +
                          " (max: " + std::to_string(caps.max_step) + ")");
    }

    std::unique_lock<std::mutex> lock(movement_mutex_);
    movement_abort_requested_ = false;

    try {
      MeasurePerformance("MoveTo",
                         [&]() { ThrowIfFailed(focuser_->Move(position)); });

      WaitForMovementComplete();
    } catch (...) {
      movement_abort_requested_ = false;
      throw;
    }
  }

  std::future<void> MoveToAsync(long position) {
    return std::async(std::launch::async,
                      [this, position]() { MoveTo(position); });
  }

  void MoveBy(long steps) {
    long current_position = GetPosition();
    long target_position = current_position + steps;
    MoveTo(target_position);
  }

  void Halt() {
    if (!GetCapabilities().can_halt) {
      movement_abort_requested_ = true;
      movement_condition_.notify_all();
      return;
    }

    MeasurePerformance("Halt", [this]() { ThrowIfFailed(focuser_->Halt()); });

    movement_abort_requested_ = true;
    movement_condition_.notify_all();
  }

  // **运动状态查询**
  bool IsMoving() const {
    return MeasurePerformance("IsMoving", [this]() {
      VARIANT_BOOL moving;
      ThrowIfFailed(focuser_->get_IsMoving(&moving));
      return moving == VARIANT_TRUE;
    });
  }

  // **温度相关**
  double GetTemperature() const {
    if (!GetCapabilities().temperature_available) {
      throw AscomException(0x80040400, "Temperature reading not available");
    }

    return MeasurePerformance("GetTemperature", [this]() {
      double temp;
      ThrowIfFailed(focuser_->get_Temperature(&temp));
      return temp;
    });
  }

  bool GetTemperatureCompensation() const {
    if (!GetCapabilities().temperature_compensation) {
      return false;
    }

    return MeasurePerformance("GetTemperatureCompensation", [this]() {
      VARIANT_BOOL enabled;
      ThrowIfFailed(focuser_->get_TempComp(&enabled));
      return enabled == VARIANT_TRUE;
    });
  }

  void SetTemperatureCompensation(bool enabled) {
    if (!GetCapabilities().temperature_compensation) {
      throw AscomException(0x80040400,
                           "Temperature compensation not available");
    }

    MeasurePerformance("SetTemperatureCompensation", [&]() {
      ThrowIfFailed(
          focuser_->put_TempComp(enabled ? VARIANT_TRUE : VARIANT_FALSE));
    });

    if (enabled && GetCapabilities().temperature_available) {
      // **初始化温度补偿参考点**
      try {
        double current_temp = GetTemperature();
        long current_pos = GetPosition();

        last_compensation_temp_ = current_temp;
        compensation_reference_position_ = current_pos;
      } catch (...) {
      }
    }
  }

  // **手动温度补偿**
  void ApplyTemperatureCompensation(const AutoFocusConfig &config) {
    if (!GetCapabilities().temperature_available) {
      return;
    }

    try {
      double current_temp = GetTemperature();
      double temp_change = current_temp - last_compensation_temp_.load();

      if (std::abs(temp_change) > 0.5) // 温度变化超过0.5度
      {
        long compensation_steps =
            static_cast<long>(temp_change * config.temp_coefficient);
        long current_pos = GetPosition();
        long target_pos = current_pos + compensation_steps;

        // **应用补偿**
        MoveTo(target_pos);

        last_compensation_temp_ = current_temp;

        std::cout << "Applied temperature compensation: " << temp_change
                  << "°C -> " << compensation_steps << " steps" << std::endl;
      }
    } catch (const AscomException &e) {
      std::cerr << "**WARNING:** Temperature compensation failed: " << e.what()
                << std::endl;
    }
  }

  // **自动调焦功能**
  struct AutoFocusResult {
    bool success = false;
    long final_position = 0;
    double final_fwhm = 0.0;
    int iterations = 0;
    std::string error_message;
    std::vector<FocusData> measurements;
  };

  AutoFocusResult
  PerformAutoFocus(const AutoFocusConfig &config,
                   std::function<double(long)> measure_quality_func) {
    AutoFocusResult result;

    if (!measure_quality_func) {
      result.error_message = "No quality measurement function provided";
      return result;
    }

    try {
      long start_position = GetPosition();
      long best_position = start_position;
      double best_quality = std::numeric_limits<double>::max();

      std::cout << "Starting auto-focus from position " << start_position
                << std::endl;

      // **粗扫描**
      std::vector<std::pair<long, double>> coarse_scan;
      long scan_start = std::max(0L, start_position - config.search_range / 2);
      long scan_end = std::min(GetCapabilities().max_step,
                               start_position + config.search_range / 2);

      for (long pos = scan_start; pos <= scan_end; pos += config.step_size) {
        if (movement_abort_requested_) {
          result.error_message = "Auto-focus aborted by user";
          return result;
        }

        MoveTo(pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 稳定时间

        double quality = measure_quality_func(pos);
        coarse_scan.emplace_back(pos, quality);

        FocusData data(pos, 0.0, quality, 0.0);
        if (GetCapabilities().temperature_available) {
          try {
            data.temperature = GetTemperature();
          } catch (...) {
          }
        }

        RecordFocusData(data);
        result.measurements.push_back(data);

        if (quality < best_quality) {
          best_quality = quality;
          best_position = pos;
        }

        std::cout << "Position: " << pos << ", Quality: " << quality
                  << std::endl;
      }

      // **精细扫描**
      if (!coarse_scan.empty()) {
        long fine_start = std::max(0L, best_position - config.step_size);
        long fine_end = std::min(GetCapabilities().max_step,
                                 best_position + config.step_size);

        for (long pos = fine_start; pos <= fine_end;
             pos += config.fine_step_size) {
          if (movement_abort_requested_) {
            result.error_message = "Auto-focus aborted during fine scan";
            break;
          }

          MoveTo(pos);
          std::this_thread::sleep_for(std::chrono::milliseconds(300));

          double quality = measure_quality_func(pos);

          FocusData data(pos, 0.0, quality, 0.0);
          if (GetCapabilities().temperature_available) {
            try {
              data.temperature = GetTemperature();
            } catch (...) {
            }
          }

          RecordFocusData(data);
          result.measurements.push_back(data);

          if (quality < best_quality) {
            best_quality = quality;
            best_position = pos;
          }

          result.iterations++;
        }
      }

      // **移动到最佳位置**
      MoveTo(best_position);

      result.success = (best_quality <= config.target_fwhm + config.tolerance);
      result.final_position = best_position;
      result.final_fwhm = best_quality;

      std::cout << "Auto-focus completed. Best position: " << best_position
                << ", Quality: " << best_quality << std::endl;
    } catch (const AscomException &e) {
      result.error_message = e.what();
    } catch (const std::exception &e) {
      result.error_message = e.what();
    }

    return result;
  }

  // **焦点历史管理**
  void RecordFocusData(const FocusData &data) {
    std::lock_guard<std::mutex> lock(history_mutex_);

    focus_history_.push_back(data);

    if (focus_history_.size() > MAX_HISTORY_SIZE) {
      focus_history_.pop_front();
    }
  }

  std::vector<FocusData> GetFocusHistory(size_t max_entries = 0) const {
    std::lock_guard<std::mutex> lock(history_mutex_);

    if (max_entries == 0 || max_entries >= focus_history_.size()) {
      return std::vector<FocusData>(focus_history_.begin(),
                                    focus_history_.end());
    }

    auto start = focus_history_.end() - static_cast<long>(max_entries);
    return std::vector<FocusData>(start, focus_history_.end());
  }

  void ClearFocusHistory() {
    std::lock_guard<std::mutex> lock(history_mutex_);
    focus_history_.clear();
  }

  // **焦点分析**
  std::optional<long> PredictOptimalPosition() const {
    auto history = GetFocusHistory(20); // 最近20个数据点

    if (history.size() < 5) {
      return std::nullopt;
    }

    // **简单的抛物线拟合**
    std::vector<std::pair<double, double>> points;
    for (const auto &data : history) {
      points.emplace_back(static_cast<double>(data.position), data.fwhm);
    }

    // **找到最小FWHM对应的位置**
    auto min_it = std::min_element(
        points.begin(), points.end(),
        [](const auto &a, const auto &b) { return a.second < b.second; });

    return static_cast<long>(min_it->first);
  }

  // **焦点曲线V-curve分析**
  struct VCurveAnalysis {
    long optimal_position = 0;
    double optimal_fwhm = 0.0;
    double curve_quality = 0.0; // 拟合质量评估
    bool reliable = false;
  };

  VCurveAnalysis AnalyzeVCurve(const std::vector<FocusData> &data) const {
    VCurveAnalysis result;

    if (data.size() < 5) {
      return result;
    }

    // **排序数据**
    auto sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end(),
              [](const FocusData &a, const FocusData &b) {
                return a.position < b.position;
              });

    // **寻找最小值**
    auto min_it = std::min_element(
        sorted_data.begin(), sorted_data.end(),
        [](const FocusData &a, const FocusData &b) { return a.fwhm < b.fwhm; });

    result.optimal_position = min_it->position;
    result.optimal_fwhm = min_it->fwhm;

    // **评估曲线质量**
    double variance = 0.0;
    for (const auto &point : sorted_data) {
      double diff = point.fwhm - result.optimal_fwhm;
      variance += diff * diff;
    }
    variance /= sorted_data.size();

    result.curve_quality = 1.0 / (1.0 + variance); // 质量评分
    result.reliable = (result.curve_quality > 0.5 && sorted_data.size() >= 7);

    return result;
  }

protected:
  void OnConnected() override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    capabilities_.loaded = false;
  }

private:
  void LoadCapabilities() const {
    capabilities_.loaded = true;

    try {
      VARIANT_BOOL bool_result;
      long long_result;
      double double_result;

      // **能力查询**
      ThrowIfFailed(focuser_->get_Absolute(&bool_result));
      capabilities_.absolute_positioning = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(focuser_->get_TempCompAvailable(&bool_result));
      capabilities_.temperature_compensation = (bool_result == VARIANT_TRUE);

      // **温度传感器可用性检测**
      try {
        ThrowIfFailed(focuser_->get_Temperature(&double_result));
        capabilities_.temperature_available = true;
      } catch (...) {
        capabilities_.temperature_available = false;
      }

      ThrowIfFailed(focuser_->get_MaxStep(&long_result));
      capabilities_.max_step = long_result;

      ThrowIfFailed(focuser_->get_MaxIncrement(&long_result));
      capabilities_.max_increment = long_result;

      try {
        ThrowIfFailed(focuser_->get_StepSize(&double_result));
        capabilities_.step_size = double_result;
      } catch (...) {
        capabilities_.step_size = 1.0; // 默认值
      }

      // **测试Halt能力**
      try {
        if (IsMoving()) {
          ThrowIfFailed(focuser_->Halt());
          capabilities_.can_halt = true;
        } else {
          capabilities_.can_halt = true; // 假设支持
        }
      } catch (...) {
        capabilities_.can_halt = false;
      }
    } catch (const AscomException &) {
      capabilities_ = FocuserCapabilities{};
      capabilities_.loaded = true;
    }
  }

  void WaitForMovementComplete() {
    const auto poll_interval = std::chrono::milliseconds(100);
    const auto timeout = std::chrono::minutes(5);
    auto start_time = std::chrono::steady_clock::now();

    while (!movement_abort_requested_) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        throw AscomException(0x80040408, "Focuser movement timeout");
      }

      try {
        if (!IsMoving()) {
          movement_condition_.notify_all();
          break;
        }
      } catch (...) {
        // **查询状态失败，继续等待**
      }

      std::this_thread::sleep_for(poll_interval);
    }

    movement_abort_requested_ = false;
  }
};