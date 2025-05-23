#pragma once
#include "AscomDriver.h"
#include <variant>

// **开关类型枚举**
enum class SwitchType : int {
  Boolean = 0, // 布尔开关
  Range = 1,   // 范围开关（滑块）
  ReadOnly = 2 // 只读状态
};

// **开关信息结构**
struct SwitchInfo {
  int id;
  std::string name;
  std::string description;
  SwitchType type;
  double minimum = 0.0;
  double maximum = 1.0;
  double step = 1.0;
  std::variant<bool, double> value;
  bool can_write = true;

  SwitchInfo(int i = 0, const std::string &n = "",
             SwitchType t = SwitchType::Boolean)
      : id(i), name(n), type(t), value(false) {}
};

// **ASCOM开关接口**
MIDL_INTERFACE("6F7A5B2E-C8D9-4E3F-A4B5-92C6F8E1A7D3")
ISwitch : public IAscomDriver {
public:
  virtual HRESULT STDMETHODCALLTYPE get_MaxSwitch(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE GetSwitch(SHORT id,
                                              VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE GetSwitchDescription(SHORT id,
                                                         BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE GetSwitchName(SHORT id, BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE GetSwitchValue(SHORT id, double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE MinSwitchValue(SHORT id, double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE MaxSwitchValue(SHORT id, double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE SetSwitch(SHORT id, VARIANT_BOOL state) = 0;
  virtual HRESULT STDMETHODCALLTYPE SetSwitchName(SHORT id, BSTR name) = 0;
  virtual HRESULT STDMETHODCALLTYPE SetSwitchValue(SHORT id, double value) = 0;
  virtual HRESULT STDMETHODCALLTYPE SwitchStep(SHORT id, double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE CanWrite(SHORT id, VARIANT_BOOL * pVal) = 0;
};

// **ASCOM开关驱动封装**
class AscomSwitch : public AscomDriverBase {
private:
  AscomPtr<ISwitch> switch_device_;

  // **开关配置缓存**
  mutable std::vector<SwitchInfo> switches_;
  mutable std::unordered_map<std::string, int> name_to_id_;
  mutable std::mutex switches_mutex_;
  mutable bool switches_loaded_ = false;

public:
  explicit AscomSwitch(const std::string &prog_id) : AscomDriverBase(prog_id) {
    switch_device_ = AscomDeviceFactory::CreateDevice<ISwitch>(prog_id);
  }

  // **开关发现和管理**
  const std::vector<SwitchInfo> &GetSwitches() const {
    std::lock_guard<std::mutex> lock(switches_mutex_);

    if (!switches_loaded_) {
      LoadSwitchConfiguration();
    }

    return switches_;
  }

  int GetSwitchCount() const { return static_cast<int>(GetSwitches().size()); }

  // **按ID操作开关**
  bool GetSwitchState(int id) const {
    ValidateSwitchId(id);

    return MeasurePerformance("GetSwitchState", [&]() {
      VARIANT_BOOL state;
      ThrowIfFailed(switch_device_->GetSwitch(static_cast<SHORT>(id), &state));
      return state == VARIANT_TRUE;
    });
  }

  void SetSwitchState(int id, bool state) {
    ValidateSwitchId(id);
    auto switches = GetSwitches();

    if (!switches[id].can_write) {
      throw AscomException(0x80040400,
                           "Switch " + std::to_string(id) + " is read-only");
    }

    MeasurePerformance("SetSwitchState", [&]() {
      ThrowIfFailed(switch_device_->SetSwitch(
          static_cast<SHORT>(id), state ? VARIANT_TRUE : VARIANT_FALSE));
    });

    // **更新缓存**
    std::lock_guard<std::mutex> lock(switches_mutex_);
    if (id < static_cast<int>(switches_.size())) {
      switches_[id].value = state;
    }
  }

  double GetSwitchValue(int id) const {
    ValidateSwitchId(id);

    return MeasurePerformance("GetSwitchValue", [&]() {
      double value;
      ThrowIfFailed(
          switch_device_->GetSwitchValue(static_cast<SHORT>(id), &value));
      return value;
    });
  }

  void SetSwitchValue(int id, double value) {
    ValidateSwitchId(id);
    auto switches = GetSwitches();

    if (!switches[id].can_write) {
      throw AscomException(0x80040400,
                           "Switch " + std::to_string(id) + " is read-only");
    }

    // **值范围检查**
    if (value < switches[id].minimum || value > switches[id].maximum) {
      throw AscomException(
          0x80040005, "Value out of range: " + std::to_string(value) +
                          " (range: " + std::to_string(switches[id].minimum) +
                          "-" + std::to_string(switches[id].maximum) + ")");
    }

    MeasurePerformance("SetSwitchValue", [&]() {
      ThrowIfFailed(
          switch_device_->SetSwitchValue(static_cast<SHORT>(id), value));
    });

    // **更新缓存**
    std::lock_guard<std::mutex> lock(switches_mutex_);
    if (id < static_cast<int>(switches_.size())) {
      switches_[id].value = value;
    }
  }

  // **按名称操作开关**
  bool GetSwitchState(const std::string &name) const {
    int id = GetSwitchIdByName(name);
    return GetSwitchState(id);
  }

  void SetSwitchState(const std::string &name, bool state) {
    int id = GetSwitchIdByName(name);
    SetSwitchState(id, state);
  }

  double GetSwitchValue(const std::string &name) const {
    int id = GetSwitchIdByName(name);
    return GetSwitchValue(id);
  }

  void SetSwitchValue(const std::string &name, double value) {
    int id = GetSwitchIdByName(name);
    SetSwitchValue(id, value);
  }

  // **批量操作**
  std::vector<bool> GetAllSwitchStates() const {
    auto switches = GetSwitches();
    std::vector<bool> states;
    states.reserve(switches.size());

    for (size_t i = 0; i < switches.size(); ++i) {
      try {
        states.push_back(GetSwitchState(static_cast<int>(i)));
      } catch (...) {
        states.push_back(false); // **默认值**
      }
    }

    return states;
  }

  void SetMultipleSwitches(const std::vector<std::pair<int, bool>> &switches) {
    for (const auto &[id, state] : switches) {
      try {
        SetSwitchState(id, state);
      } catch (const AscomException &e) {
        std::cerr << "**ERROR** setting switch " << id << ": " << e.what()
                  << std::endl;
      }
    }
  }

  // **场景管理**
  struct SwitchScene {
    std::string name;
    std::vector<std::pair<int, std::variant<bool, double>>> switch_settings;
    std::string description;
  };

  void ApplyScene(const SwitchScene &scene) {
    std::cout << "Applying scene: " << scene.name << std::endl;

    for (const auto &[id, setting] : scene.switch_settings) {
      try {
        if (std::holds_alternative<bool>(setting)) {
          SetSwitchState(id, std::get<bool>(setting));
        } else if (std::holds_alternative<double>(setting)) {
          SetSwitchValue(id, std::get<double>(setting));
        }
      } catch (const AscomException &e) {
        std::cerr << "**ERROR** applying scene setting for switch " << id
                  << ": " << e.what() << std::endl;
      }
    }
  }

  SwitchScene CreateCurrentScene(const std::string &name) const {
    SwitchScene scene;
    scene.name = name;
    scene.description =
        "Captured on " + std::to_string(std::chrono::system_clock::to_time_t(
                             std::chrono::system_clock::now()));

    auto switches = GetSwitches();
    for (size_t i = 0; i < switches.size(); ++i) {
      try {
        if (switches[i].type == SwitchType::Boolean) {
          bool state = GetSwitchState(static_cast<int>(i));
          scene.switch_settings.emplace_back(static_cast<int>(i), state);
        } else {
          double value = GetSwitchValue(static_cast<int>(i));
          scene.switch_settings.emplace_back(static_cast<int>(i), value);
        }
      } catch (...) {
        // **跳过无法读取的开关**
      }
    }

    return scene;
  }

  // **监控功能**
  void StartSwitchMonitoring(
      std::chrono::milliseconds interval,
      std::function<void(int, std::variant<bool, double>)> callback) {
    if (!callback) {
      throw std::invalid_argument("Callback function required");
    }

    monitoring_active_ = true;
    monitoring_thread_ = std::make_unique<std::thread>(
        [this, interval, callback]() { MonitoringLoop(interval, callback); });
  }

  void StopSwitchMonitoring() {
    monitoring_active_ = false;
    if (monitoring_thread_ && monitoring_thread_->joinable()) {
      monitoring_thread_->join();
      monitoring_thread_.reset();
    }
  }

protected:
  void OnConnected() override {
    std::lock_guard<std::mutex> lock(switches_mutex_);
    switches_loaded_ = false;
  }

private:
  std::unique_ptr<std::thread> monitoring_thread_;
  std::atomic<bool> monitoring_active_{false};

  void LoadSwitchConfiguration() const {
    switches_loaded_ = true;
    switches_.clear();
    name_to_id_.clear();

    try {
      SHORT max_switch;
      ThrowIfFailed(switch_device_->get_MaxSwitch(&max_switch));

      for (SHORT i = 0; i < max_switch; ++i) {
        SwitchInfo info;
        info.id = i;

        // **获取开关名称**
        BSTR name;
        ThrowIfFailed(switch_device_->GetSwitchName(i, &name));
        if (name) {
          _bstr_t wrapper(name, false);
          info.name = wrapper;
        } else {
          info.name = "Switch " + std::to_string(i);
        }

        // **获取描述**
        BSTR desc;
        ThrowIfFailed(switch_device_->GetSwitchDescription(i, &desc));
        if (desc) {
          _bstr_t wrapper(desc, false);
          info.description = wrapper;
        }

        // **获取范围信息**
        double min_val, max_val, step_val;
        ThrowIfFailed(switch_device_->MinSwitchValue(i, &min_val));
        ThrowIfFailed(switch_device_->MaxSwitchValue(i, &max_val));
        ThrowIfFailed(switch_device_->SwitchStep(i, &step_val));

        info.minimum = min_val;
        info.maximum = max_val;
        info.step = step_val;

        // **确定开关类型**
        if (min_val == 0.0 && max_val == 1.0 && step_val == 1.0) {
          info.type = SwitchType::Boolean;
        } else {
          info.type = SwitchType::Range;
        }

        // **检查可写性**
        VARIANT_BOOL can_write;
        ThrowIfFailed(switch_device_->CanWrite(i, &can_write));
        info.can_write = (can_write == VARIANT_TRUE);

        if (!info.can_write) {
          info.type = SwitchType::ReadOnly;
        }

        switches_.push_back(info);
        name_to_id_[info.name] = i;
      }
    } catch (const AscomException &) {
      switches_.clear();
      name_to_id_.clear();
    }
  }

  void ValidateSwitchId(int id) const {
    auto switches = GetSwitches();
    if (id < 0 || id >= static_cast<int>(switches.size())) {
      throw AscomException(0x80040005,
                           "Invalid switch ID: " + std::to_string(id));
    }
  }

  int GetSwitchIdByName(const std::string &name) const {
    auto switches = GetSwitches();
    auto it = name_to_id_.find(name);
    if (it == name_to_id_.end()) {
      throw AscomException(0x80040005, "Switch not found: " + name);
    }
    return it->second;
  }

  void MonitoringLoop(
      std::chrono::milliseconds interval,
      std::function<void(int, std::variant<bool, double>)> callback) {
    // **记录上次的值用于变化检测**
    std::vector<std::variant<bool, double>> last_values;
    auto switches = GetSwitches();
    last_values.resize(switches.size());

    // **初始化上次值**
    for (size_t i = 0; i < switches.size(); ++i) {
      try {
        if (switches[i].type == SwitchType::Boolean) {
          last_values[i] = GetSwitchState(static_cast<int>(i));
        } else {
          last_values[i] = GetSwitchValue(static_cast<int>(i));
        }
      } catch (...) {
        last_values[i] = false; // **默认值**
      }
    }

    while (monitoring_active_) {
      try {
        for (size_t i = 0; i < switches.size(); ++i) {
          std::variant<bool, double> current_value;

          if (switches[i].type == SwitchType::Boolean) {
            current_value = GetSwitchState(static_cast<int>(i));
          } else {
            current_value = GetSwitchValue(static_cast<int>(i));
          }

          // **检查值是否变化**
          if (current_value != last_values[i]) {
            callback(static_cast<int>(i), current_value);
            last_values[i] = current_value;
          }
        }
      } catch (const AscomException &e) {
        std::cerr << "**WARNING:** Switch monitoring error: " << e.what()
                  << std::endl;
      } catch (...) {
        // **忽略其他异常**
      }

      std::this_thread::sleep_for(interval);
    }
  }
};