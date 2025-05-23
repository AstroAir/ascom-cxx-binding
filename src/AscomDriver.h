#pragma once
#include "AscomSmartPtr.h"
#include <chrono>
#include <future>
#include <map>


// **前向声明ASCOM接口**
MIDL_INTERFACE("76618F90-032F-4424-94BA-4451385ABD5C")
IAscomDriver : public IDispatch {
public:
  virtual HRESULT STDMETHODCALLTYPE get_Connected(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_Connected(VARIANT_BOOL newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Description(BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_DriverInfo(BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_DriverVersion(BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_InterfaceVersion(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Name(BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE SetupDialog() = 0;
  virtual HRESULT STDMETHODCALLTYPE Action(
      BSTR ActionName, BSTR ActionParameters, BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE CommandBlind(BSTR Command,
                                                 VARIANT_BOOL Raw) = 0;
  virtual HRESULT STDMETHODCALLTYPE CommandBool(BSTR Command, VARIANT_BOOL Raw,
                                                VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE CommandString(
      BSTR Command, VARIANT_BOOL Raw, BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE Dispose() = 0;
};

// **ASCOM基础驱动封装类**
class AscomDriverBase {
protected:
  AscomPtr<IAscomDriver> driver_;
  std::string prog_id_;
  std::string device_name_;
  mutable std::shared_mutex state_mutex_;
  std::atomic<AscomDeviceState> current_state_{AscomDeviceState::Disconnected};

  // **性能追踪**
  mutable std::map<std::string, std::chrono::microseconds> performance_metrics_;
  mutable std::mutex metrics_mutex_;

public:
  explicit AscomDriverBase(const std::string &prog_id) : prog_id_(prog_id) {
    driver_ = AscomDeviceFactory::CreateDevice<IAscomDriver>(prog_id);
    LoadDeviceInfo();
  }

  virtual ~AscomDriverBase() {
    try {
      if (IsConnected()) {
        Disconnect();
      }
    } catch (...) {
      // **析构函数中不抛出异常**
    }
  }

  // **基础属性访问**
  std::string GetDescription() const {
    return GetBSTRProperty(
        [this]() {
          BSTR result;
          ThrowIfFailed(driver_->get_Description(&result));
          return result;
        },
        "GetDescription");
  }

  std::string GetDriverInfo() const {
    return GetBSTRProperty(
        [this]() {
          BSTR result;
          ThrowIfFailed(driver_->get_DriverInfo(&result));
          return result;
        },
        "GetDriverInfo");
  }

  std::string GetDriverVersion() const {
    return GetBSTRProperty(
        [this]() {
          BSTR result;
          ThrowIfFailed(driver_->get_DriverVersion(&result));
          return result;
        },
        "GetDriverVersion");
  }

  short GetInterfaceVersion() const {
    return MeasurePerformance("GetInterfaceVersion", [this]() {
      SHORT result;
      ThrowIfFailed(driver_->get_InterfaceVersion(&result));
      return result;
    });
  }

  std::string GetName() const { return device_name_; }

  // **连接管理**
  virtual void Connect() {
    std::unique_lock<std::shared_mutex> lock(state_mutex_);

    if (current_state_ == AscomDeviceState::Connected)
      return;

    MeasurePerformance("Connect", [this]() {
      ThrowIfFailed(driver_->put_Connected(VARIANT_TRUE));
    });

    current_state_ = AscomDeviceState::Connected;
    OnConnected();
  }

  virtual void Disconnect() {
    std::unique_lock<std::shared_mutex> lock(state_mutex_);

    if (current_state_ == AscomDeviceState::Disconnected)
      return;

    try {
      OnDisconnecting();

      MeasurePerformance("Disconnect", [this]() {
        ThrowIfFailed(driver_->put_Connected(VARIANT_FALSE));
      });

      current_state_ = AscomDeviceState::Disconnected;
    } catch (...) {
      current_state_ = AscomDeviceState::Error;
      throw;
    }
  }

  bool IsConnected() const {
    std::shared_lock<std::shared_mutex> lock(state_mutex_);

    if (current_state_ != AscomDeviceState::Connected)
      return false;

    try {
      VARIANT_BOOL connected;
      ThrowIfFailed(driver_->get_Connected(&connected));
      return connected == VARIANT_TRUE;
    } catch (...) {
      return false;
    }
  }

  AscomDeviceState GetDeviceState() const noexcept {
    return current_state_.load();
  }

  // **设备配置**
  void ShowSetupDialog() {
    MeasurePerformance("ShowSetupDialog",
                       [this]() { ThrowIfFailed(driver_->SetupDialog()); });
  }

  // **执行命令**
  std::string ExecuteAction(const std::string &action_name,
                            const std::string &parameters = "") {
    return GetBSTRProperty(
        [&]() {
          BSTR result;
          _bstr_t name(action_name.c_str());
          _bstr_t params(parameters.c_str());
          ThrowIfFailed(driver_->Action(name, params, &result));
          return result;
        },
        "ExecuteAction");
  }

  void SendBlindCommand(const std::string &command, bool raw = false) {
    MeasurePerformance("SendBlindCommand", [&]() {
      _bstr_t cmd(command.c_str());
      ThrowIfFailed(
          driver_->CommandBlind(cmd, raw ? VARIANT_TRUE : VARIANT_FALSE));
    });
  }

  bool SendBoolCommand(const std::string &command, bool raw = false) {
    return MeasurePerformance("SendBoolCommand", [&]() {
      VARIANT_BOOL result;
      _bstr_t cmd(command.c_str());
      ThrowIfFailed(driver_->CommandBool(
          cmd, raw ? VARIANT_TRUE : VARIANT_FALSE, &result));
      return result == VARIANT_TRUE;
    });
  }

  std::string SendStringCommand(const std::string &command, bool raw = false) {
    return GetBSTRProperty(
        [&]() {
          BSTR result;
          _bstr_t cmd(command.c_str());
          ThrowIfFailed(driver_->CommandString(
              cmd, raw ? VARIANT_TRUE : VARIANT_FALSE, &result));
          return result;
        },
        "SendStringCommand");
  }

  // **性能指标获取**
  std::map<std::string, std::chrono::microseconds>
  GetPerformanceMetrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return performance_metrics_;
  }

  void ClearPerformanceMetrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    performance_metrics_.clear();
  }

protected:
  // **子类可重写的钩子方法**
  virtual void OnConnected() {}
  virtual void OnDisconnecting() {}

  // **错误处理辅助函数**
  static void ThrowIfFailed(HRESULT hr,
                            const std::string &operation = "ASCOM operation") {
    if (FAILED(hr)) {
      _com_error error(hr);
      std::string message =
          operation + " failed: " + std::string(error.ErrorMessage());
      throw AscomException(hr, message, operation);
    }
  }

  // **BSTR属性获取辅助函数**
  template <typename Func>
  std::string GetBSTRProperty(Func &&func, const std::string &operation) const {
    return MeasurePerformance(operation, [&]() {
      BSTR result = func();
      if (!result)
        return std::string();

      _bstr_t wrapper(result, false); // 不复制，直接接管
      return std::string(wrapper);
    });
  }

  // **性能测量辅助函数**
  template <typename Func>
  auto MeasurePerformance(const std::string &operation, Func &&func) const
      -> std::invoke_result_t<Func> {
    auto start = std::chrono::high_resolution_clock::now();

    try {
      if constexpr (std::is_void_v<std::invoke_result_t<Func>>) {
        func();
        auto end = std::chrono::high_resolution_clock::now();
        RecordPerformance(operation, end - start);
      } else {
        auto result = func();
        auto end = std::chrono::high_resolution_clock::now();
        RecordPerformance(operation, end - start);
        return result;
      }
    } catch (...) {
      auto end = std::chrono::high_resolution_clock::now();
      RecordPerformance(operation + "_ERROR", end - start);
      throw;
    }
  }

  void RecordPerformance(const std::string &operation,
                         std::chrono::nanoseconds duration) const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    auto microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(duration);
    performance_metrics_[operation] = microseconds;
  }

private:
  void LoadDeviceInfo() {
    try {
      device_name_ = GetBSTRProperty(
          [this]() {
            BSTR result;
            ThrowIfFailed(driver_->get_Name(&result));
            return result;
          },
          "LoadDeviceInfo");
    } catch (const AscomException &) {
      device_name_ = prog_id_; // 降级使用ProgID
    }
  }
};