#pragma once
#include "AscomCamera.h"
#include "AscomDome.h"
#include "AscomFilterWheel.h"
#include "AscomFocuser.h"
#include "AscomRotator.h"
#include "AscomSwitch.h"
#include "AscomTelescope.h"


// **设备连接状态**
enum class DeviceConnectionState {
  Disconnected,
  Connecting,
  Connected,
  Disconnecting,
  Error
};

// **设备类型枚举**
enum class DeviceType {
  Telescope,
  Camera,
  FilterWheel,
  Focuser,
  Dome,
  Rotator,
  Switch
};

// **设备信息结构**
struct DeviceInfo {
  DeviceType type;
  std::string prog_id;
  std::string name;
  std::string description;
  DeviceConnectionState state = DeviceConnectionState::Disconnected;
  std::chrono::system_clock::time_point last_activity;
  std::string last_error;

  DeviceInfo(DeviceType t, const std::string &pid)
      : type(t), prog_id(pid), last_activity(std::chrono::system_clock::now()) {
  }
};

// **ASCOM设备管理器**
class AscomDeviceManager {
private:
  // **设备实例存储**
  std::unique_ptr<AscomTelescope> telescope_;
  std::unique_ptr<AscomCamera> camera_;
  std::unique_ptr<AscomFilterWheel> filter_wheel_;
  std::unique_ptr<AscomFocuser> focuser_;
  std::unique_ptr<AscomDome> dome_;
  std::unique_ptr<AscomRotator> rotator_;
  std::unique_ptr<AscomSwitch> switch_device_;

  // **设备注册表**
  std::vector<DeviceInfo> device_registry_;
  std::mutex registry_mutex_;

  // **事件回调**
  std::function<void(DeviceType, DeviceConnectionState)> connection_callback_;
  std::function<void(DeviceType, const std::string &)> error_callback_;

  // **自动重连**
  std::unique_ptr<std::thread> watchdog_thread_;
  std::atomic<bool> watchdog_active_{false};
  std::chrono::seconds reconnect_interval_{30};

public:
  AscomDeviceManager() = default;

  ~AscomDeviceManager() {
    DisconnectAllDevices();
    StopWatchdog();
  }

  // **设备发现**
  std::vector<std::string> DiscoverDevices(DeviceType type) const {
    std::string device_type_str = DeviceTypeToString(type);
    return GetRegisteredProgIDs(device_type_str);
  }

  // **设备连接管理**
  void ConnectTelescope(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::Telescope,
                        DeviceConnectionState::Connecting);

      telescope_ = std::make_unique<AscomTelescope>(prog_id);
      telescope_->Connect();

      RegisterDevice(DeviceType::Telescope, prog_id, telescope_->GetName());
      UpdateDeviceState(DeviceType::Telescope,
                        DeviceConnectionState::Connected);

      std::cout << "**Telescope connected:** " << telescope_->GetName()
                << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::Telescope, e.what());
      telescope_.reset();
      throw;
    }
  }

  void ConnectCamera(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::Camera, DeviceConnectionState::Connecting);

      camera_ = std::make_unique<AscomCamera>(prog_id);
      camera_->Connect();

      RegisterDevice(DeviceType::Camera, prog_id, camera_->GetName());
      UpdateDeviceState(DeviceType::Camera, DeviceConnectionState::Connected);

      std::cout << "**Camera connected:** " << camera_->GetName() << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::Camera, e.what());
      camera_.reset();
      throw;
    }
  }

  void ConnectFilterWheel(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::FilterWheel,
                        DeviceConnectionState::Connecting);

      filter_wheel_ = std::make_unique<AscomFilterWheel>(prog_id);
      filter_wheel_->Connect();

      RegisterDevice(DeviceType::FilterWheel, prog_id,
                     filter_wheel_->GetName());
      UpdateDeviceState(DeviceType::FilterWheel,
                        DeviceConnectionState::Connected);

      std::cout << "**Filter Wheel connected:** " << filter_wheel_->GetName()
                << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::FilterWheel, e.what());
      filter_wheel_.reset();
      throw;
    }
  }

  void ConnectFocuser(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::Focuser, DeviceConnectionState::Connecting);

      focuser_ = std::make_unique<AscomFocuser>(prog_id);
      focuser_->Connect();

      RegisterDevice(DeviceType::Focuser, prog_id, focuser_->GetName());
      UpdateDeviceState(DeviceType::Focuser, DeviceConnectionState::Connected);

      std::cout << "**Focuser connected:** " << focuser_->GetName()
                << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::Focuser, e.what());
      focuser_.reset();
      throw;
    }
  }

  void ConnectDome(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::Dome, DeviceConnectionState::Connecting);

      dome_ = std::make_unique<AscomDome>(prog_id);
      dome_->Connect();

      RegisterDevice(DeviceType::Dome, prog_id, dome_->GetName());
      UpdateDeviceState(DeviceType::Dome, DeviceConnectionState::Connected);

      // **如果有望远镜，设置圆顶跟踪**
      if (telescope_ && dome_->GetCapabilities().can_slave) {
        dome_->EnableSlaveMode(
            [this]() { return telescope_->GetCurrentAltAz(); });
        std::cout << "Dome slave tracking enabled" << std::endl;
      }

      std::cout << "**Dome connected:** " << dome_->GetName() << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::Dome, e.what());
      dome_.reset();
      throw;
    }
  }

  void ConnectRotator(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::Rotator, DeviceConnectionState::Connecting);

      rotator_ = std::make_unique<AscomRotator>(prog_id);
      rotator_->Connect();

      RegisterDevice(DeviceType::Rotator, prog_id, rotator_->GetName());
      UpdateDeviceState(DeviceType::Rotator, DeviceConnectionState::Connected);

      std::cout << "**Rotator connected:** " << rotator_->GetName()
                << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::Rotator, e.what());
      rotator_.reset();
      throw;
    }
  }

  void ConnectSwitch(const std::string &prog_id) {
    try {
      UpdateDeviceState(DeviceType::Switch, DeviceConnectionState::Connecting);

      switch_device_ = std::make_unique<AscomSwitch>(prog_id);
      switch_device_->Connect();

      RegisterDevice(DeviceType::Switch, prog_id, switch_device_->GetName());
      UpdateDeviceState(DeviceType::Switch, DeviceConnectionState::Connected);

      std::cout << "**Switch connected:** " << switch_device_->GetName()
                << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(DeviceType::Switch, e.what());
      switch_device_.reset();
      throw;
    }
  }

  // **设备访问器**
  AscomTelescope *GetTelescope() const { return telescope_.get(); }
  AscomCamera *GetCamera() const { return camera_.get(); }
  AscomFilterWheel *GetFilterWheel() const { return filter_wheel_.get(); }
  AscomFocuser *GetFocuser() const { return focuser_.get(); }
  AscomDome *GetDome() const { return dome_.get(); }
  AscomRotator *GetRotator() const { return rotator_.get(); }
  AscomSwitch *GetSwitch() const { return switch_device_.get(); }

  // **设备状态查询**
  bool IsDeviceConnected(DeviceType type) const {
    switch (type) {
    case DeviceType::Telescope:
      return telescope_ && telescope_->IsConnected();
    case DeviceType::Camera:
      return camera_ && camera_->IsConnected();
    case DeviceType::FilterWheel:
      return filter_wheel_ && filter_wheel_->IsConnected();
    case DeviceType::Focuser:
      return focuser_ && focuser_->IsConnected();
    case DeviceType::Dome:
      return dome_ && dome_->IsConnected();
    case DeviceType::Rotator:
      return rotator_ && rotator_->IsConnected();
    case DeviceType::Switch:
      return switch_device_ && switch_device_->IsConnected();
    default:
      return false;
    }
  }

  DeviceConnectionState GetDeviceState(DeviceType type) const {
    std::lock_guard<std::mutex> lock(registry_mutex_);

    auto it = std::find_if(
        device_registry_.begin(), device_registry_.end(),
        [type](const DeviceInfo &info) { return info.type == type; });

    if (it != device_registry_.end()) {
      return it->state;
    }

    return DeviceConnectionState::Disconnected;
  }

  // **批量操作**
  void DisconnectAllDevices() {
    std::vector<DeviceType> types = {
        DeviceType::Switch,   DeviceType::Rotator,     DeviceType::Dome,
        DeviceType::Focuser,  DeviceType::FilterWheel, DeviceType::Camera,
        DeviceType::Telescope};

    for (auto type : types) {
      try {
        DisconnectDevice(type);
      } catch (const AscomException &e) {
        std::cerr << "**ERROR** disconnecting " << DeviceTypeToString(type)
                  << ": " << e.what() << std::endl;
      }
    }
  }

  void DisconnectDevice(DeviceType type) {
    UpdateDeviceState(type, DeviceConnectionState::Disconnecting);

    try {
      switch (type) {
      case DeviceType::Telescope:
        if (telescope_) {
          telescope_->Disconnect();
          telescope_.reset();
        }
        break;
      case DeviceType::Camera:
        if (camera_) {
          camera_->Disconnect();
          camera_.reset();
        }
        break;
      case DeviceType::FilterWheel:
        if (filter_wheel_) {
          filter_wheel_->Disconnect();
          filter_wheel_.reset();
        }
        break;
      case DeviceType::Focuser:
        if (focuser_) {
          focuser_->Disconnect();
          focuser_.reset();
        }
        break;
      case DeviceType::Dome:
        if (dome_) {
          dome_->Disconnect();
          dome_.reset();
        }
        break;
      case DeviceType::Rotator:
        if (rotator_) {
          rotator_->Disconnect();
          rotator_.reset();
        }
        break;
      case DeviceType::Switch:
        if (switch_device_) {
          switch_device_->Disconnect();
          switch_device_.reset();
        }
        break;
      }

      UpdateDeviceState(type, DeviceConnectionState::Disconnected);
      std::cout << "**" << DeviceTypeToString(type) << " disconnected**"
                << std::endl;
    } catch (const AscomException &e) {
      HandleDeviceError(type, e.what());
      throw;
    }
  }

  // **事件回调设置**
  void SetConnectionCallback(
      std::function<void(DeviceType, DeviceConnectionState)> callback) {
    connection_callback_ = std::move(callback);
  }

  void SetErrorCallback(
      std::function<void(DeviceType, const std::string &)> callback) {
    error_callback_ = std::move(callback);
  }

  // **设备注册表管理**
  std::vector<DeviceInfo> GetDeviceRegistry() const {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    return device_registry_;
  }

  // **设备健康监控**
  void StartWatchdog() {
    StopWatchdog();

    watchdog_active_ = true;
    watchdog_thread_ =
        std::make_unique<std::thread>([this]() { WatchdogLoop(); });
  }

  void StopWatchdog() {
    watchdog_active_ = false;
    if (watchdog_thread_ && watchdog_thread_->joinable()) {
      watchdog_thread_->join();
      watchdog_thread_.reset();
    }
  }

  void SetReconnectInterval(std::chrono::seconds interval) {
    reconnect_interval_ = interval;
  }

  // **性能报告**
  struct SystemPerformanceReport {
    std::map<DeviceType, std::map<std::string, std::chrono::microseconds>>
        device_metrics;
    std::chrono::system_clock::time_point timestamp;
    size_t total_operations = 0;
    size_t failed_operations = 0;

    SystemPerformanceReport() { timestamp = std::chrono::system_clock::now(); }
  };

  SystemPerformanceReport GeneratePerformanceReport() const {
    SystemPerformanceReport report;

    if (telescope_) {
      report.device_metrics[DeviceType::Telescope] =
          telescope_->GetPerformanceMetrics();
    }

    if (camera_) {
      report.device_metrics[DeviceType::Camera] =
          camera_->GetPerformanceMetrics();
    }

    if (filter_wheel_) {
      report.device_metrics[DeviceType::FilterWheel] =
          filter_wheel_->GetPerformanceMetrics();
    }

    if (focuser_) {
      report.device_metrics[DeviceType::Focuser] =
          focuser_->GetPerformanceMetrics();
    }

    if (dome_) {
      report.device_metrics[DeviceType::Dome] = dome_->GetPerformanceMetrics();
    }

    if (rotator_) {
      report.device_metrics[DeviceType::Rotator] =
          rotator_->GetPerformanceMetrics();
    }

    if (switch_device_) {
      report.device_metrics[DeviceType::Switch] =
          switch_device_->GetPerformanceMetrics();
    }

    return report;
  }

private:
  static std::string DeviceTypeToString(DeviceType type) {
    switch (type) {
    case DeviceType::Telescope:
      return "Telescope";
    case DeviceType::Camera:
      return "Camera";
    case DeviceType::FilterWheel:
      return "FilterWheel";
    case DeviceType::Focuser:
      return "Focuser";
    case DeviceType::Dome:
      return "Dome";
    case DeviceType::Rotator:
      return "Rotator";
    case DeviceType::Switch:
      return "Switch";
    default:
      return "Unknown";
    }
  }

  std::vector<std::string>
  GetRegisteredProgIDs(const std::string &device_type) const {
    std::vector<std::string> prog_ids;

    HKEY key;
    std::string subkey = "SOFTWARE\\ASCOM\\" + device_type;

    if (RegOpenKeyEx(HKEY_LOCAL_MACHINE, subkey.c_str(), 0, KEY_READ, &key) ==
        ERROR_SUCCESS) {
      DWORD index = 0;
      char name[256];
      DWORD name_size;

      while (true) {
        name_size = sizeof(name);
        if (RegEnumKeyEx(key, index, name, &name_size, nullptr, nullptr,
                         nullptr, nullptr) != ERROR_SUCCESS)
          break;

        prog_ids.emplace_back(name);
        index++;
      }

      RegCloseKey(key);
    }

    return prog_ids;
  }

  void RegisterDevice(DeviceType type, const std::string &prog_id,
                      const std::string &name) {
    std::lock_guard<std::mutex> lock(registry_mutex_);

    // **移除已存在的设备**
    device_registry_.erase(std::remove_if(device_registry_.begin(),
                                          device_registry_.end(),
                                          [type](const DeviceInfo &info) {
                                            return info.type == type;
                                          }),
                           device_registry_.end());

    // **添加新设备**
    DeviceInfo info(type, prog_id);
    info.name = name;
    info.state = DeviceConnectionState::Connected;
    info.last_activity = std::chrono::system_clock::now();

    device_registry_.push_back(info);
  }

  void UpdateDeviceState(DeviceType type, DeviceConnectionState state) {
    std::lock_guard<std::mutex> lock(registry_mutex_);

    auto it =
        std::find_if(device_registry_.begin(), device_registry_.end(),
                     [type](DeviceInfo &info) { return info.type == type; });

    if (it != device_registry_.end()) {
      it->state = state;
      it->last_activity = std::chrono::system_clock::now();
    }

    // **触发回调**
    if (connection_callback_) {
      try {
        connection_callback_(type, state);
      } catch (...) {
        // **忽略回调异常**
      }
    }
  }

  void HandleDeviceError(DeviceType type, const std::string &error) {
    std::lock_guard<std::mutex> lock(registry_mutex_);

    auto it =
        std::find_if(device_registry_.begin(), device_registry_.end(),
                     [type](DeviceInfo &info) { return info.type == type; });

    if (it != device_registry_.end()) {
      it->state = DeviceConnectionState::Error;
      it->last_error = error;
      it->last_activity = std::chrono::system_clock::now();
    }

    // **触发错误回调**
    if (error_callback_) {
      try {
        error_callback_(type, error);
      } catch (...) {
        // **忽略回调异常**
      }
    }

    std::cerr << "**ERROR** in " << DeviceTypeToString(type) << ": " << error
              << std::endl;
  }

  void WatchdogLoop() {
    while (watchdog_active_) {
      try {
        // **检查所有连接的设备状态**
        std::vector<DeviceType> types_to_check;

        {
          std::lock_guard<std::mutex> lock(registry_mutex_);
          for (const auto &device : device_registry_) {
            if (device.state == DeviceConnectionState::Connected) {
              types_to_check.push_back(device.type);
            }
          }
        }

        for (auto type : types_to_check) {
          if (!IsDeviceConnected(type)) {
            std::cout << "**Watchdog detected disconnection:** "
                      << DeviceTypeToString(type) << std::endl;

            UpdateDeviceState(type, DeviceConnectionState::Error);

            // **这里可以添加自动重连逻辑**
          }
        }
      } catch (...) {
        // **忽略看门狗异常**
      }

      std::this_thread::sleep_for(reconnect_interval_);
    }
  }
};