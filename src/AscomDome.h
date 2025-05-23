#pragma once
#include "AscomDriver.h"

// **圆顶状态枚举**
enum class DomeShutterState : int {
  Open = 0,
  Closed = 1,
  Opening = 2,
  Closing = 3,
  Error = 4
};

// **ASCOM圆顶接口定义**
MIDL_INTERFACE("4D8B5FA0-B6C7-4E8D-92F3-84A5C16E79DF")
IDome : public IAscomDriver {
public:
  virtual HRESULT STDMETHODCALLTYPE AbortSlew() = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Altitude(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_AtHome(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_AtPark(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Azimuth(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanFindHome(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanPark(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSetAltitude(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSetAzimuth(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSetPark(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSetShutter(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSlave(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSyncAzimuth(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE CloseShutter() = 0;
  virtual HRESULT STDMETHODCALLTYPE FindHome() = 0;
  virtual HRESULT STDMETHODCALLTYPE OpenShutter() = 0;
  virtual HRESULT STDMETHODCALLTYPE Park() = 0;
  virtual HRESULT STDMETHODCALLTYPE SetPark() = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ShutterStatus(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Slaved(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_Slaved(VARIANT_BOOL newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Slewing(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE SlewToAltitude(double Altitude) = 0;
  virtual HRESULT STDMETHODCALLTYPE SlewToAzimuth(double Azimuth) = 0;
  virtual HRESULT STDMETHODCALLTYPE SyncToAzimuth(double Azimuth) = 0;
};

// **圆顶能力结构**
struct DomeCapabilities {
  bool can_find_home = false;
  bool can_park = false;
  bool can_set_altitude = false;
  bool can_set_azimuth = false;
  bool can_set_park = false;
  bool can_set_shutter = false;
  bool can_slave = false;
  bool can_sync_azimuth = false;

  bool loaded = false;
};

// **圆顶状态信息**
struct DomeStatus {
  double azimuth = 0.0;
  double altitude = 0.0;
  bool at_home = false;
  bool at_park = false;
  bool slewing = false;
  bool slaved = false;
  DomeShutterState shutter_state = DomeShutterState::Closed;
  std::chrono::system_clock::time_point last_update;

  DomeStatus() { last_update = std::chrono::system_clock::now(); }
};

// **ASCOM圆顶驱动封装**
class AscomDome : public AscomDriverBase {
private:
  AscomPtr<IDome> dome_;

  // **能力缓存**
  mutable DomeCapabilities capabilities_;
  mutable std::mutex capabilities_mutex_;

  // **状态缓存**
  mutable DomeStatus cached_status_;
  mutable std::mutex status_mutex_;
  mutable std::chrono::steady_clock::time_point last_status_update_;
  static constexpr std::chrono::milliseconds STATUS_CACHE_DURATION{500};

  // **运动和快门控制**
  std::atomic<bool> slew_abort_requested_{false};
  std::atomic<bool> shutter_abort_requested_{false};
  mutable std::mutex operation_mutex_;
  std::condition_variable operation_condition_;

  // **同步跟踪**
  std::unique_ptr<std::thread> slave_tracking_thread_;
  std::atomic<bool> slave_tracking_active_{false};
  std::function<AltAzCoordinates()> telescope_position_provider_;

public:
  explicit AscomDome(const std::string &prog_id) : AscomDriverBase(prog_id) {
    dome_ = AscomDeviceFactory::CreateDevice<IDome>(prog_id);
    last_status_update_ = std::chrono::steady_clock::time_point{};
  }

  ~AscomDome() {
    try {
      StopSlaveTracking();
    } catch (...) {
    }
  }

  // **能力查询**
  const DomeCapabilities &GetCapabilities() const {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);

    if (!capabilities_.loaded) {
      LoadCapabilities();
    }

    return capabilities_;
  }

  // **状态查询**
  DomeStatus GetStatus(bool force_refresh = false) const {
    std::lock_guard<std::mutex> lock(status_mutex_);

    auto now = std::chrono::steady_clock::now();
    if (force_refresh || (now - last_status_update_) > STATUS_CACHE_DURATION) {
      RefreshStatus();
      last_status_update_ = now;
    }

    return cached_status_;
  }

  // **方位角控制**
  double GetAzimuth() const { return GetStatus().azimuth; }

  void SlewToAzimuth(double azimuth) {
    if (!GetCapabilities().can_set_azimuth) {
      throw AscomException(0x80040400, "Azimuth control not supported");
    }

    // **规范化方位角到0-360度**
    azimuth = std::fmod(azimuth, 360.0);
    if (azimuth < 0)
      azimuth += 360.0;

    std::unique_lock<std::mutex> lock(operation_mutex_);
    slew_abort_requested_ = false;

    try {
      MeasurePerformance("SlewToAzimuth", [&]() {
        ThrowIfFailed(dome_->SlewToAzimuth(azimuth));
      });

      WaitForSlewComplete();
    } catch (...) {
      slew_abort_requested_ = false;
      throw;
    }
  }

  std::future<void> SlewToAzimuthAsync(double azimuth) {
    return std::async(std::launch::async,
                      [this, azimuth]() { SlewToAzimuth(azimuth); });
  }

  void SyncToAzimuth(double azimuth) {
    if (!GetCapabilities().can_sync_azimuth) {
      throw AscomException(0x80040400, "Azimuth sync not supported");
    }

    MeasurePerformance("SyncToAzimuth",
                       [&]() { ThrowIfFailed(dome_->SyncToAzimuth(azimuth)); });
  }

  // **高度角控制**
  double GetAltitude() const { return GetStatus().altitude; }

  void SlewToAltitude(double altitude) {
    if (!GetCapabilities().can_set_altitude) {
      throw AscomException(0x80040400, "Altitude control not supported");
    }

    if (altitude < 0.0 || altitude > 90.0) {
      throw AscomException(0x80040005, "Altitude out of range: " +
                                           std::to_string(altitude));
    }

    std::unique_lock<std::mutex> lock(operation_mutex_);
    slew_abort_requested_ = false;

    try {
      MeasurePerformance("SlewToAltitude", [&]() {
        ThrowIfFailed(dome_->SlewToAltitude(altitude));
      });

      WaitForSlewComplete();
    } catch (...) {
      slew_abort_requested_ = false;
      throw;
    }
  }

  std::future<void> SlewToAltitudeAsync(double altitude) {
    return std::async(std::launch::async,
                      [this, altitude]() { SlewToAltitude(altitude); });
  }

  // **快门控制**
  DomeShutterState GetShutterState() const { return GetStatus().shutter_state; }

  void OpenShutter() {
    if (!GetCapabilities().can_set_shutter) {
      throw AscomException(0x80040400, "Shutter control not supported");
    }

    std::unique_lock<std::mutex> lock(operation_mutex_);
    shutter_abort_requested_ = false;

    try {
      MeasurePerformance("OpenShutter",
                         [this]() { ThrowIfFailed(dome_->OpenShutter()); });

      WaitForShutterOperation(DomeShutterState::Open);
    } catch (...) {
      shutter_abort_requested_ = false;
      throw;
    }
  }

  void CloseShutter() {
    if (!GetCapabilities().can_set_shutter) {
      throw AscomException(0x80040400, "Shutter control not supported");
    }

    std::unique_lock<std::mutex> lock(operation_mutex_);
    shutter_abort_requested_ = false;

    try {
      MeasurePerformance("CloseShutter",
                         [this]() { ThrowIfFailed(dome_->CloseShutter()); });

      WaitForShutterOperation(DomeShutterState::Closed);
    } catch (...) {
      shutter_abort_requested_ = false;
      throw;
    }
  }

  std::future<void> OpenShutterAsync() {
    return std::async(std::launch::async, [this]() { OpenShutter(); });
  }

  std::future<void> CloseShutterAsync() {
    return std::async(std::launch::async, [this]() { CloseShutter(); });
  }

  // **运动控制**
  void AbortSlew() {
    MeasurePerformance("AbortSlew",
                       [this]() { ThrowIfFailed(dome_->AbortSlew()); });

    slew_abort_requested_ = true;
    operation_condition_.notify_all();
  }

  bool IsSlewing() const { return GetStatus().slewing; }

  // **停放功能**
  void Park() {
    if (!GetCapabilities().can_park) {
      throw AscomException(0x80040400, "Parking not supported");
    }

    std::unique_lock<std::mutex> lock(operation_mutex_);
    slew_abort_requested_ = false;

    try {
      MeasurePerformance("Park", [this]() { ThrowIfFailed(dome_->Park()); });

      WaitForSlewComplete();
    } catch (...) {
      slew_abort_requested_ = false;
      throw;
    }
  }

  void SetParkPosition() {
    if (!GetCapabilities().can_set_park) {
      throw AscomException(0x80040400, "Set park position not supported");
    }

    MeasurePerformance("SetParkPosition",
                       [this]() { ThrowIfFailed(dome_->SetPark()); });
  }

  void FindHome() {
    if (!GetCapabilities().can_find_home) {
      throw AscomException(0x80040400, "Home finding not supported");
    }

    std::unique_lock<std::mutex> lock(operation_mutex_);
    slew_abort_requested_ = false;

    try {
      MeasurePerformance("FindHome",
                         [this]() { ThrowIfFailed(dome_->FindHome()); });

      WaitForSlewComplete();
    } catch (...) {
      slew_abort_requested_ = false;
      throw;
    }
  }

  bool IsAtHome() const { return GetStatus().at_home; }

  bool IsAtPark() const { return GetStatus().at_park; }

  // **同步跟踪功能**
  void EnableSlaveMode(std::function<AltAzCoordinates()> position_provider) {
    if (!GetCapabilities().can_slave) {
      throw AscomException(0x80040400, "Slave mode not supported");
    }

    if (!position_provider) {
      throw std::invalid_argument("Position provider function required");
    }

    telescope_position_provider_ = std::move(position_provider);

    MeasurePerformance("EnableSlaveMode", [this]() {
      ThrowIfFailed(dome_->put_Slaved(VARIANT_TRUE));
    });

    StartSlaveTracking();
  }

  void DisableSlaveMode() {
    StopSlaveTracking();

    if (GetCapabilities().can_slave) {
      MeasurePerformance("DisableSlaveMode", [this]() {
        ThrowIfFailed(dome_->put_Slaved(VARIANT_FALSE));
      });
    }
  }

  bool IsSlaved() const { return GetStatus().slaved; }

  // **同步跟踪配置**
  struct SlaveTrackingConfig {
    double azimuth_tolerance = 5.0;                  // 方位角误差容限（度）
    std::chrono::milliseconds update_interval{2000}; // 更新间隔
    double minimum_move = 1.0;                       // 最小移动量（度）
    bool track_when_slewing = false;                 // 望远镜转动时是否跟踪
  };

  void ConfigureSlaveTracking(const SlaveTrackingConfig &config) {
    slave_config_ = config;
  }

  // **状态监控和诊断**
  struct DiagnosticInfo {
    std::chrono::milliseconds avg_slew_time{0};
    std::chrono::milliseconds avg_shutter_time{0};
    size_t successful_operations = 0;
    size_t failed_operations = 0;
    double position_accuracy = 0.0; // 位置精度统计
    std::string last_error;
  };

  DiagnosticInfo GetDiagnosticInfo() const {
    // **返回诊断信息实现**
    DiagnosticInfo info;
    auto metrics = GetPerformanceMetrics();

    if (metrics.count("SlewToAzimuth"))
      info.avg_slew_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              metrics.at("SlewToAzimuth"));

    if (metrics.count("OpenShutter"))
      info.avg_shutter_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              metrics.at("OpenShutter"));

    return info;
  }

protected:
  void OnConnected() override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    capabilities_.loaded = false;
  }

  void OnDisconnecting() override { StopSlaveTracking(); }

private:
  SlaveTrackingConfig slave_config_;

  void LoadCapabilities() const {
    capabilities_.loaded = true;

    try {
      VARIANT_BOOL bool_result;

      ThrowIfFailed(dome_->get_CanFindHome(&bool_result));
      capabilities_.can_find_home = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanPark(&bool_result));
      capabilities_.can_park = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanSetAltitude(&bool_result));
      capabilities_.can_set_altitude = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanSetAzimuth(&bool_result));
      capabilities_.can_set_azimuth = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanSetPark(&bool_result));
      capabilities_.can_set_park = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanSetShutter(&bool_result));
      capabilities_.can_set_shutter = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanSlave(&bool_result));
      capabilities_.can_slave = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_CanSyncAzimuth(&bool_result));
      capabilities_.can_sync_azimuth = (bool_result == VARIANT_TRUE);
    } catch (const AscomException &) {
      capabilities_ = DomeCapabilities{};
      capabilities_.loaded = true;
    }
  }

  void RefreshStatus() const {
    try {
      double double_result;
      VARIANT_BOOL bool_result;
      SHORT short_result;

      ThrowIfFailed(dome_->get_Azimuth(&double_result));
      cached_status_.azimuth = double_result;

      ThrowIfFailed(dome_->get_Altitude(&double_result));
      cached_status_.altitude = double_result;

      ThrowIfFailed(dome_->get_AtHome(&bool_result));
      cached_status_.at_home = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_AtPark(&bool_result));
      cached_status_.at_park = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(dome_->get_Slewing(&bool_result));
      cached_status_.slewing = (bool_result == VARIANT_TRUE);

      if (GetCapabilities().can_slave) {
        ThrowIfFailed(dome_->get_Slaved(&bool_result));
        cached_status_.slaved = (bool_result == VARIANT_TRUE);
      }

      if (GetCapabilities().can_set_shutter) {
        ThrowIfFailed(dome_->get_ShutterStatus(&short_result));
        cached_status_.shutter_state =
            static_cast<DomeShutterState>(short_result);
      }

      cached_status_.last_update = std::chrono::system_clock::now();
    } catch (const AscomException &) {
      // **状态查询失败，保持之前的状态**
    }
  }

  void WaitForSlewComplete() {
    const auto poll_interval = std::chrono::milliseconds(200);
    const auto timeout = std::chrono::minutes(10);
    auto start_time = std::chrono::steady_clock::now();

    while (!slew_abort_requested_) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        throw AscomException(0x80040408, "Dome slew timeout");
      }

      try {
        if (!IsSlewing()) {
          operation_condition_.notify_all();
          break;
        }
      } catch (...) {
        // **查询状态失败，继续等待**
      }

      std::this_thread::sleep_for(poll_interval);
    }

    slew_abort_requested_ = false;
  }

  void WaitForShutterOperation(DomeShutterState target_state) {
    const auto poll_interval = std::chrono::milliseconds(500);
    const auto timeout = std::chrono::minutes(5);
    auto start_time = std::chrono::steady_clock::now();

    while (!shutter_abort_requested_) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        throw AscomException(0x80040408, "Shutter operation timeout");
      }

      try {
        auto current_state = GetShutterState();
        if (current_state == target_state) {
          operation_condition_.notify_all();
          break;
        } else if (current_state == DomeShutterState::Error) {
          throw AscomException(0x80040006, "Shutter error state");
        }
      } catch (...) {
        // **查询状态失败，继续等待**
      }

      std::this_thread::sleep_for(poll_interval);
    }

    shutter_abort_requested_ = false;
  }

  void StartSlaveTracking() {
    StopSlaveTracking();

    slave_tracking_active_ = true;
    slave_tracking_thread_ =
        std::make_unique<std::thread>([this]() { SlaveTrackingLoop(); });
  }

  void StopSlaveTracking() {
    slave_tracking_active_ = false;

    if (slave_tracking_thread_ && slave_tracking_thread_->joinable()) {
      slave_tracking_thread_->join();
      slave_tracking_thread_.reset();
    }
  }

  void SlaveTrackingLoop() {
    while (slave_tracking_active_) {
      try {
        if (telescope_position_provider_) {
          auto telescope_pos = telescope_position_provider_();
          auto dome_status = GetStatus();

          // **计算方位角差异**
          double azimuth_diff = telescope_pos.azimuth - dome_status.azimuth;

          // **处理360度边界**
          if (azimuth_diff > 180.0)
            azimuth_diff -= 360.0;
          if (azimuth_diff < -180.0)
            azimuth_diff += 360.0;

          // **检查是否需要移动**
          if (std::abs(azimuth_diff) > slave_config_.azimuth_tolerance &&
              std::abs(azimuth_diff) > slave_config_.minimum_move) {
            if (!dome_status.slewing || slave_config_.track_when_slewing) {
              // **执行跟踪移动**
              SlewToAzimuthAsync(telescope_pos.azimuth);
            }
          }
        }
      } catch (const AscomException &e) {
        std::cerr << "**WARNING:** Slave tracking error: " << e.what()
                  << std::endl;
      } catch (...) {
        // **忽略其他异常，继续跟踪**
      }

      std::this_thread::sleep_for(slave_config_.update_interval);
    }
  }
};