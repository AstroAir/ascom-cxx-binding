#pragma once
#include "AscomDriver.h"
#include <array>
#include <unordered_map>

// **ASCOM滤镜轮接口定义**
MIDL_INTERFACE("2E9F2F6A-7D4E-4D8B-A92C-793B16BFE847")
IFilterWheel : public IAscomDriver {
public:
  virtual HRESULT STDMETHODCALLTYPE get_FocusOffsets(SAFEARRAY * *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Names(SAFEARRAY * *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Position(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_Position(SHORT newVal) = 0;
};

// **滤镜信息结构**
struct FilterInfo {
  std::string name;
  int position;
  int focus_offset;  // 焦点偏移量（步数）
  std::string color; // 颜色描述
  double wavelength; // 中心波长（纳米）
  double bandwidth;  // 带宽（纳米）

  FilterInfo(const std::string &n = "", int pos = -1, int offset = 0)
      : name(n), position(pos), focus_offset(offset), wavelength(0.0),
        bandwidth(0.0) {}
};

// **滤镜轮驱动封装**
class AscomFilterWheel : public AscomDriverBase {
private:
  AscomPtr<IFilterWheel> filter_wheel_;

  // **滤镜配置缓存**
  mutable std::vector<FilterInfo> filters_;
  mutable std::unordered_map<std::string, int> name_to_position_;
  mutable std::mutex filter_cache_mutex_;
  mutable bool cache_loaded_ = false;

  // **运动状态跟踪**
  std::atomic<bool> is_moving_{false};
  mutable std::mutex movement_mutex_;
  std::condition_variable movement_condition_;

public:
  explicit AscomFilterWheel(const std::string &prog_id)
      : AscomDriverBase(prog_id) {
    filter_wheel_ = AscomDeviceFactory::CreateDevice<IFilterWheel>(prog_id);
  }

  // **滤镜配置加载与管理**
  const std::vector<FilterInfo> &GetFilters() const {
    std::lock_guard<std::mutex> lock(filter_cache_mutex_);

    if (!cache_loaded_) {
      LoadFilterConfiguration();
    }

    return filters_;
  }

  void SetFilterConfiguration(const std::vector<FilterInfo> &filters) {
    std::lock_guard<std::mutex> lock(filter_cache_mutex_);

    filters_ = filters;
    name_to_position_.clear();

    for (const auto &filter : filters_) {
      if (filter.position >= 0) {
        name_to_position_[filter.name] = filter.position;
      }
    }

    cache_loaded_ = true;
  }

  // **滤镜位置控制**
  int GetCurrentPosition() const {
    return MeasurePerformance("GetCurrentPosition", [this]() {
      SHORT position;
      ThrowIfFailed(filter_wheel_->get_Position(&position));
      return static_cast<int>(position);
    });
  }

  void SetPosition(int position) {
    auto filters = GetFilters();
    if (position < 0 || position >= static_cast<int>(filters.size())) {
      throw AscomException(0x80040005, "Invalid filter position: " +
                                           std::to_string(position));
    }

    std::unique_lock<std::mutex> lock(movement_mutex_);
    is_moving_ = true;

    try {
      MeasurePerformance("SetPosition", [&]() {
        ThrowIfFailed(
            filter_wheel_->put_Position(static_cast<SHORT>(position)));
      });

      // **等待运动完成**
      WaitForMovementComplete();
    } catch (...) {
      is_moving_ = false;
      throw;
    }
  }

  std::future<void> SetPositionAsync(int position) {
    return std::async(std::launch::async,
                      [this, position]() { SetPosition(position); });
  }

  // **按名称选择滤镜**
  void SelectFilter(const std::string &name) {
    auto filters = GetFilters();

    auto it = name_to_position_.find(name);
    if (it == name_to_position_.end()) {
      throw AscomException(0x80040005, "Filter not found: " + name);
    }

    SetPosition(it->second);
  }

  std::future<void> SelectFilterAsync(const std::string &name) {
    return std::async(std::launch::async,
                      [this, name]() { SelectFilter(name); });
  }

  // **当前滤镜信息**
  FilterInfo GetCurrentFilter() const {
    int position = GetCurrentPosition();
    auto filters = GetFilters();

    if (position >= 0 && position < static_cast<int>(filters.size())) {
      return filters[position];
    }

    return FilterInfo("Unknown", position, 0);
  }

  // **滤镜列表查询**
  std::vector<std::string> GetFilterNames() const {
    auto filters = GetFilters();
    std::vector<std::string> names;
    names.reserve(filters.size());

    for (const auto &filter : filters) {
      names.push_back(filter.name);
    }

    return names;
  }

  // **焦点偏移**
  std::vector<int> GetFocusOffsets() const {
    return MeasurePerformance("GetFocusOffsets", [this]() {
      SAFEARRAY *offsets_array = nullptr;
      ThrowIfFailed(filter_wheel_->get_FocusOffsets(&offsets_array));

      std::vector<int> offsets;
      if (offsets_array) {
        offsets = SafeArrayToIntVector(offsets_array);
        SafeArrayDestroy(offsets_array);
      }

      return offsets;
    });
  }

  int GetFocusOffset(int position) const {
    auto filters = GetFilters();
    if (position >= 0 && position < static_cast<int>(filters.size())) {
      return filters[position].focus_offset;
    }
    return 0;
  }

  int GetFocusOffset(const std::string &name) const {
    auto it = name_to_position_.find(name);
    if (it != name_to_position_.end()) {
      return GetFocusOffset(it->second);
    }
    return 0;
  }

  // **运动状态查询**
  bool IsMoving() const noexcept { return is_moving_.load(); }

  // **高级功能**
  void CreateFilterSequence(
      const std::vector<std::string> &filter_names,
      std::function<void(const std::string &)> callback = nullptr) {
    for (const auto &filter_name : filter_names) {
      try {
        SelectFilter(filter_name);

        if (callback) {
          callback(filter_name);
        }
      } catch (const AscomException &e) {
        std::cerr << "**ERROR** selecting filter " << filter_name << ": "
                  << e.what() << std::endl;
        throw;
      }
    }
  }

  // **配置管理**
  void SaveFilterConfiguration(const std::string &filename) const {
    auto filters = GetFilters();

    std::ofstream file(filename);
    if (!file) {
      throw std::runtime_error("Cannot create filter configuration file: " +
                               filename);
    }

    file << "# Filter Configuration\n";
    file << "# Format: Position,Name,FocusOffset,Color,Wavelength,Bandwidth\n";

    for (const auto &filter : filters) {
      file << filter.position << "," << filter.name << ","
           << filter.focus_offset << "," << filter.color << ","
           << filter.wavelength << "," << filter.bandwidth << "\n";
    }
  }

  void LoadFilterConfiguration(const std::string &filename) {
    std::ifstream file(filename);
    if (!file) {
      throw std::runtime_error("Cannot open filter configuration file: " +
                               filename);
    }

    std::vector<FilterInfo> new_filters;
    std::string line;

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#')
        continue;

      std::stringstream ss(line);
      std::string token;
      FilterInfo filter;

      // 解析CSV格式
      if (std::getline(ss, token, ','))
        filter.position = std::stoi(token);
      if (std::getline(ss, token, ','))
        filter.name = token;
      if (std::getline(ss, token, ','))
        filter.focus_offset = std::stoi(token);
      if (std::getline(ss, token, ','))
        filter.color = token;
      if (std::getline(ss, token, ','))
        filter.wavelength = std::stod(token);
      if (std::getline(ss, token, ','))
        filter.bandwidth = std::stod(token);

      new_filters.push_back(filter);
    }

    SetFilterConfiguration(new_filters);
  }

protected:
  void OnConnected() override {
    std::lock_guard<std::mutex> lock(filter_cache_mutex_);
    cache_loaded_ = false;
  }

private:
  void LoadFilterConfiguration() const {
    cache_loaded_ = true;
    filters_.clear();
    name_to_position_.clear();

    try {
      // **获取滤镜名称**
      SAFEARRAY *names_array = nullptr;
      ThrowIfFailed(filter_wheel_->get_Names(&names_array));

      std::vector<std::string> names;
      if (names_array) {
        names = SafeArrayToStringVector(names_array);
        SafeArrayDestroy(names_array);
      }

      // **获取焦点偏移**
      auto offsets = GetFocusOffsets();

      // **创建滤镜列表**
      for (size_t i = 0; i < names.size(); ++i) {
        FilterInfo filter;
        filter.position = static_cast<int>(i);
        filter.name = names[i];
        filter.focus_offset = (i < offsets.size()) ? offsets[i] : 0;

        filters_.push_back(filter);
        name_to_position_[filter.name] = filter.position;
      }
    } catch (const AscomException &) {
      // **如果无法获取配置，创建默认配置**
      for (int i = 0; i < 8; ++i) // 假设8位滤镜轮
      {
        FilterInfo filter;
        filter.position = i;
        filter.name = "Filter " + std::to_string(i + 1);
        filter.focus_offset = 0;

        filters_.push_back(filter);
        name_to_position_[filter.name] = filter.position;
      }
    }
  }

  void WaitForMovementComplete() {
    const auto poll_interval = std::chrono::milliseconds(100);
    const auto timeout = std::chrono::minutes(2);
    auto start_time = std::chrono::steady_clock::now();

    int target_position = -1;
    try {
      SHORT pos;
      ThrowIfFailed(filter_wheel_->get_Position(&pos));
      target_position = pos;
    } catch (...) {
    }

    while (is_moving_) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        is_moving_ = false;
        throw AscomException(0x80040408, "Filter wheel movement timeout");
      }

      try {
        int current_position = GetCurrentPosition();
        if (target_position >= 0 && current_position == target_position) {
          is_moving_ = false;
          movement_condition_.notify_all();
          break;
        }
      } catch (...) {
        // **查询状态失败，继续等待**
      }

      std::this_thread::sleep_for(poll_interval);
    }
  }

  std::vector<std::string> SafeArrayToStringVector(SAFEARRAY *psa) const {
    std::vector<std::string> result;
    if (!psa)
      return result;

    LONG lBound, uBound;
    ThrowIfFailed(SafeArrayGetLBound(psa, 1, &lBound));
    ThrowIfFailed(SafeArrayGetUBound(psa, 1, &uBound));

    for (LONG i = lBound; i <= uBound; ++i) {
      BSTR bstr;
      ThrowIfFailed(SafeArrayGetElement(psa, &i, &bstr));

      if (bstr) {
        _bstr_t wrapper(bstr, false);
        result.emplace_back(wrapper);
      }
    }

    return result;
  }

  std::vector<int> SafeArrayToIntVector(SAFEARRAY *psa) const {
    std::vector<int> result;
    if (!psa)
      return result;

    LONG lBound, uBound;
    ThrowIfFailed(SafeArrayGetLBound(psa, 1, &lBound));
    ThrowIfFailed(SafeArrayGetUBound(psa, 1, &uBound));

    for (LONG i = lBound; i <= uBound; ++i) {
      VARIANT var;
      VariantInit(&var);

      ThrowIfFailed(SafeArrayGetElement(psa, &i, &var));

      if (var.vt == VT_I2)
        result.push_back(var.iVal);
      else if (var.vt == VT_I4)
        result.push_back(var.lVal);

      VariantClear(&var);
    }

    return result;
  }
};
