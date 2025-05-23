#pragma once
#include "AscomDriver.h"
#include <condition_variable>
#include <future>

MIDL_INTERFACE("0C88F828-0E09-4C6F-A85D-731A8B860824")
ITelescope : public IAscomDriver
{
public:
    virtual HRESULT STDMETHODCALLTYPE AbortSlew() = 0;
    virtual HRESULT STDMETHODCALLTYPE get_AlignmentMode(SHORT* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Altitude(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_ApertureArea(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_ApertureDiameter(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_AtHome(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_AtPark(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Azimuth(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanFindHome(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanPark(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanPulseGuide(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSetDeclinationRate(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSetGuideRates(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSetPark(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSetPierSide(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSetRightAscensionRate(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSetTracking(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSlew(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSlewAltAz(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSlewAltAzAsync(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSlewAsync(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSync(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanSyncAltAz(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_CanUnpark(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Declination(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_DeclinationRate(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_DeclinationRate(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_DoesRefraction(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_DoesRefraction(VARIANT_BOOL newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_EquatorialSystem(SHORT* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE FindHome() = 0;
    virtual HRESULT STDMETHODCALLTYPE get_FocalLength(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_GuideRateDeclination(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_GuideRateDeclination(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_GuideRateRightAscension(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_GuideRateRightAscension(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_IsPulseGuiding(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE MoveAxis(SHORT Axis, double Rate) = 0;
    virtual HRESULT STDMETHODCALLTYPE Park() = 0;
    virtual HRESULT STDMETHODCALLTYPE PulseGuide(SHORT Direction, long Duration) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_RightAscension(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_RightAscensionRate(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_RightAscensionRate(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE SetPark() = 0;
    virtual HRESULT STDMETHODCALLTYPE get_SideOfPier(SHORT* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_SideOfPier(SHORT newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_SiderealTime(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_SiteElevation(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_SiteElevation(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_SiteLatitude(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_SiteLatitude(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_SiteLongitude(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_SiteLongitude(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Slewing(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE SlewToAltAz(double Azimuth, double Altitude) = 0;
    virtual HRESULT STDMETHODCALLTYPE SlewToAltAzAsync(double Azimuth, double Altitude) = 0;
    virtual HRESULT STDMETHODCALLTYPE SlewToCoordinates(double RightAscension, double Declination) = 0;
    virtual HRESULT STDMETHODCALLTYPE SlewToCoordinatesAsync(double RightAscension, double Declination) = 0;
    virtual HRESULT STDMETHODCALLTYPE SlewToTarget() = 0;
    virtual HRESULT STDMETHODCALLTYPE SlewToTargetAsync() = 0;
    virtual HRESULT STDMETHODCALLTYPE SyncToAltAz(double Azimuth, double Altitude) = 0;
    virtual HRESULT STDMETHODCALLTYPE SyncToCoordinates(double RightAscension, double Declination) = 0;
    virtual HRESULT STDMETHODCALLTYPE SyncToTarget() = 0;
    virtual HRESULT STDMETHODCALLTYPE get_TargetDeclination(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_TargetDeclination(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_TargetRightAscension(double* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_TargetRightAscension(double newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Tracking(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_Tracking(VARIANT_BOOL newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_TrackingRate(SHORT* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_TrackingRate(SHORT newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_TrackingRates(SAFEARRAY** pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE Unpark() = 0;
    virtual HRESULT STDMETHODCALLTYPE get_UTCDate(DATE* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_UTCDate(DATE newVal) = 0;
};

// **望远镜轴枚举**
enum class TelescopeAxis : short {
  Primary = 0,  // 赤经轴
  Secondary = 1 // 赤纬轴
};

// **Guide方向枚举**
enum class GuideDirection : short { North = 0, South = 1, East = 2, West = 3 };

// **跟踪速率枚举**
enum class TrackingRate : short {
  Sidereal = 0,
  Lunar = 1,
  Solar = 2,
  King = 3
};

// **ASCOM望远镜驱动封装**
class AscomTelescope : public AscomDriverBase {
private:
  AscomPtr<ITelescope> telescope_;

  // **异步操作支持**
  mutable std::mutex async_mutex_;
  std::condition_variable slew_condition_;
  std::atomic<bool> is_slewing_{false};

  // **缓存的能力属性 - 避免重复查询**
  struct TelescopeCapabilities {
    bool can_slew = false;
    bool can_slew_async = false;
    bool can_sync = false;
    bool can_park = false;
    bool can_unpark = false;
    bool can_find_home = false;
    bool can_pulse_guide = false;
    bool can_set_tracking = false;
    bool can_slew_altaz = false;
    bool can_sync_altaz = false;

    bool loaded = false;
  } capabilities_;

  mutable std::mutex capabilities_mutex_;

public:
  explicit AscomTelescope(const std::string &prog_id)
      : AscomDriverBase(prog_id) {
    telescope_ = AscomDeviceFactory::CreateDevice<ITelescope>(prog_id);
  }

  // **位置属性**
  EquatorialCoordinates GetCurrentPosition() const {
    return MeasurePerformance("GetCurrentPosition", [this]() {
      double ra, dec;
      ThrowIfFailed(telescope_->get_RightAscension(&ra));
      ThrowIfFailed(telescope_->get_Declination(&dec));
      return EquatorialCoordinates(ra, dec);
    });
  }

  AltAzCoordinates GetCurrentAltAz() const {
    return MeasurePerformance("GetCurrentAltAz", [this]() {
      double alt, az;
      ThrowIfFailed(telescope_->get_Altitude(&alt));
      ThrowIfFailed(telescope_->get_Azimuth(&az));
      return AltAzCoordinates(alt, az);
    });
  }

  EquatorialCoordinates GetTargetPosition() const {
    return MeasurePerformance("GetTargetPosition", [this]() {
      double ra, dec;
      ThrowIfFailed(telescope_->get_TargetRightAscension(&ra));
      ThrowIfFailed(telescope_->get_TargetDeclination(&dec));
      return EquatorialCoordinates(ra, dec);
    });
  }

  void SetTargetPosition(const EquatorialCoordinates &coords) {
    MeasurePerformance("SetTargetPosition", [&]() {
      ThrowIfFailed(
          telescope_->put_TargetRightAscension(coords.right_ascension));
      ThrowIfFailed(telescope_->put_TargetDeclination(coords.declination));
    });
  }

  // **状态查询**
  bool IsSlewing() const {
    return MeasurePerformance("IsSlewing", [this]() {
      VARIANT_BOOL slewing;
      ThrowIfFailed(telescope_->get_Slewing(&slewing));
      return slewing == VARIANT_TRUE;
    });
  }

  bool IsTracking() const {
    return MeasurePerformance("IsTracking", [this]() {
      VARIANT_BOOL tracking;
      ThrowIfFailed(telescope_->get_Tracking(&tracking));
      return tracking == VARIANT_TRUE;
    });
  }

  bool IsAtPark() const {
    return MeasurePerformance("IsAtPark", [this]() {
      VARIANT_BOOL at_park;
      ThrowIfFailed(telescope_->get_AtPark(&at_park));
      return at_park == VARIANT_TRUE;
    });
  }

  bool IsAtHome() const {
    return MeasurePerformance("IsAtHome", [this]() {
      VARIANT_BOOL at_home;
      ThrowIfFailed(telescope_->get_AtHome(&at_home));
      return at_home == VARIANT_TRUE;
    });
  }

  TelescopeAxisState GetAxisState() const {
    if (IsSlewing())
      return TelescopeAxisState::Slewing;
    if (IsAtPark())
      return TelescopeAxisState::Parked;
    if (IsTracking())
      return TelescopeAxisState::Tracking;
    return TelescopeAxisState::Idle;
  }

  // **运动控制 - 同步版本**
  void SlewToCoordinates(const EquatorialCoordinates &coords) {
    EnsureCapability([this]() { return GetCapabilities().can_slew; },
                     "Slewing");

    MeasurePerformance("SlewToCoordinates", [&]() {
      ThrowIfFailed(telescope_->SlewToCoordinates(coords.right_ascension,
                                                  coords.declination));
    });
  }

  void SlewToTarget() {
    EnsureCapability([this]() { return GetCapabilities().can_slew; },
                     "Slewing");

    MeasurePerformance("SlewToTarget",
                       [this]() { ThrowIfFailed(telescope_->SlewToTarget()); });
  }

  void SlewToAltAz(const AltAzCoordinates &coords) {
    EnsureCapability([this]() { return GetCapabilities().can_slew_altaz; },
                     "AltAz slewing");

    MeasurePerformance("SlewToAltAz", [&]() {
      ThrowIfFailed(telescope_->SlewToAltAz(coords.azimuth, coords.altitude));
    });
  }

  // **运动控制 - 异步版本**
  std::future<void>
  SlewToCoordinatesAsync(const EquatorialCoordinates &coords) {
    EnsureCapability([this]() { return GetCapabilities().can_slew_async; },
                     "Async slewing");

    return std::async(std::launch::async, [this, coords]() {
      std::unique_lock<std::mutex> lock(async_mutex_);
      is_slewing_ = true;

      try {
        MeasurePerformance("SlewToCoordinatesAsync", [&]() {
          ThrowIfFailed(telescope_->SlewToCoordinatesAsync(
              coords.right_ascension, coords.declination));
        });

        // **等待slew完成**
        WaitForSlewComplete();
      } catch (...) {
        is_slewing_ = false;
        throw;
      }
    });
  }

  std::future<void> SlewToTargetAsync() {
    EnsureCapability([this]() { return GetCapabilities().can_slew_async; },
                     "Async slewing");

    return std::async(std::launch::async, [this]() {
      std::unique_lock<std::mutex> lock(async_mutex_);
      is_slewing_ = true;

      try {
        MeasurePerformance("SlewToTargetAsync", [this]() {
          ThrowIfFailed(telescope_->SlewToTargetAsync());
        });

        WaitForSlewComplete();
      } catch (...) {
        is_slewing_ = false;
        throw;
      }
    });
  }

  void AbortSlew() {
    MeasurePerformance("AbortSlew",
                       [this]() { ThrowIfFailed(telescope_->AbortSlew()); });

    std::unique_lock<std::mutex> lock(async_mutex_);
    is_slewing_ = false;
    slew_condition_.notify_all();
  }

  // **同步操作**
  void SyncToCoordinates(const EquatorialCoordinates &coords) {
    EnsureCapability([this]() { return GetCapabilities().can_sync; },
                     "Syncing");

    MeasurePerformance("SyncToCoordinates", [&]() {
      ThrowIfFailed(telescope_->SyncToCoordinates(coords.right_ascension,
                                                  coords.declination));
    });
  }

  void SyncToTarget() {
    EnsureCapability([this]() { return GetCapabilities().can_sync; },
                     "Syncing");

    MeasurePerformance("SyncToTarget",
                       [this]() { ThrowIfFailed(telescope_->SyncToTarget()); });
  }

  void SyncToAltAz(const AltAzCoordinates &coords) {
    EnsureCapability([this]() { return GetCapabilities().can_sync_altaz; },
                     "AltAz syncing");

    MeasurePerformance("SyncToAltAz", [&]() {
      ThrowIfFailed(telescope_->SyncToAltAz(coords.azimuth, coords.altitude));
    });
  }

  // **跟踪控制**
  void SetTracking(bool enabled) {
    EnsureCapability([this]() { return GetCapabilities().can_set_tracking; },
                     "Tracking control");

    MeasurePerformance("SetTracking", [&]() {
      ThrowIfFailed(
          telescope_->put_Tracking(enabled ? VARIANT_TRUE : VARIANT_FALSE));
    });
  }

  TrackingRate GetTrackingRate() const {
    return MeasurePerformance("GetTrackingRate", [this]() {
      SHORT rate;
      ThrowIfFailed(telescope_->get_TrackingRate(&rate));
      return static_cast<TrackingRate>(rate);
    });
  }

  void SetTrackingRate(TrackingRate rate) {
    MeasurePerformance("SetTrackingRate", [&]() {
      ThrowIfFailed(telescope_->put_TrackingRate(static_cast<SHORT>(rate)));
    });
  }

  // **导星功能**
  void PulseGuide(GuideDirection direction,
                  std::chrono::milliseconds duration) {
    EnsureCapability([this]() { return GetCapabilities().can_pulse_guide; },
                     "Pulse guiding");

    MeasurePerformance("PulseGuide", [&]() {
      ThrowIfFailed(telescope_->PulseGuide(
          static_cast<SHORT>(direction), static_cast<long>(duration.count())));
    });
  }

  bool IsPulseGuiding() const {
    return MeasurePerformance("IsPulseGuiding", [this]() {
      VARIANT_BOOL guiding;
      ThrowIfFailed(telescope_->get_IsPulseGuiding(&guiding));
      return guiding == VARIANT_TRUE;
    });
  }

  // **轴运动控制**
  void MoveAxis(TelescopeAxis axis, double rate) {
    MeasurePerformance("MoveAxis", [&]() {
      ThrowIfFailed(telescope_->MoveAxis(static_cast<SHORT>(axis), rate));
    });
  }

  // **停放功能**
  void Park() {
    EnsureCapability([this]() { return GetCapabilities().can_park; },
                     "Parking");

    MeasurePerformance("Park", [this]() { ThrowIfFailed(telescope_->Park()); });
  }

  void Unpark() {
    EnsureCapability([this]() { return GetCapabilities().can_unpark; },
                     "Unparking");

    MeasurePerformance("Unpark",
                       [this]() { ThrowIfFailed(telescope_->Unpark()); });
  }

  void SetParkPosition() {
    EnsureCapability([this]() { return GetCapabilities().can_park; },
                     "Park position setting");

    MeasurePerformance("SetParkPosition",
                       [this]() { ThrowIfFailed(telescope_->SetPark()); });
  }

  void FindHome() {
    EnsureCapability([this]() { return GetCapabilities().can_find_home; },
                     "Home finding");

    MeasurePerformance("FindHome",
                       [this]() { ThrowIfFailed(telescope_->FindHome()); });
  }

  // **能力查询**
  const TelescopeCapabilities &GetCapabilities() const {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);

    if (!capabilities_.loaded) {
      LoadCapabilities();
    }

    return capabilities_;
  }

  // **站点信息**
  struct SiteInfo {
    double latitude;  // 纬度 (度)
    double longitude; // 经度 (度)
    double elevation; // 海拔 (米)
  };

  SiteInfo GetSiteInfo() const {
    return MeasurePerformance("GetSiteInfo", [this]() {
      SiteInfo info;
      ThrowIfFailed(telescope_->get_SiteLatitude(&info.latitude));
      ThrowIfFailed(telescope_->get_SiteLongitude(&info.longitude));
      ThrowIfFailed(telescope_->get_SiteElevation(&info.elevation));
      return info;
    });
  }

  void SetSiteInfo(const SiteInfo &info) {
    MeasurePerformance("SetSiteInfo", [&]() {
      ThrowIfFailed(telescope_->put_SiteLatitude(info.latitude));
      ThrowIfFailed(telescope_->put_SiteLongitude(info.longitude));
      ThrowIfFailed(telescope_->put_SiteElevation(info.elevation));
    });
  }

  // **时间信息**
  double GetSiderealTime() const {
    return MeasurePerformance("GetSiderealTime", [this]() {
      double time;
      ThrowIfFailed(telescope_->get_SiderealTime(&time));
      return time;
    });
  }

  std::chrono::system_clock::time_point GetUTCTime() const {
    return MeasurePerformance("GetUTCTime", [this]() {
      DATE date;
      ThrowIfFailed(telescope_->get_UTCDate(&date));

      // **将DATE转换为std::chrono::time_point**
      SYSTEMTIME st;
      VariantTimeToSystemTime(date, &st);

      FILETIME ft;
      SystemTimeToFileTime(&st, &ft);

      ULARGE_INTEGER uli;
      uli.LowPart = ft.dwLowDateTime;
      uli.HighPart = ft.dwHighDateTime;

      // **FILETIME起始于1601年1月1日，转换为Unix纪元**
      auto duration =
          std::chrono::nanoseconds((uli.QuadPart - 116444736000000000LL) * 100);
      return std::chrono::system_clock::time_point(duration);
    });
  }

  void SetUTCTime(const std::chrono::system_clock::time_point &time) {
    MeasurePerformance("SetUTCTime", [&]() {
      auto duration = time.time_since_epoch();
      auto file_time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(duration)
                  .count() /
              100 +
          116444736000000000LL;

      ULARGE_INTEGER uli;
      uli.QuadPart = file_time;

      FILETIME ft;
      ft.dwLowDateTime = uli.LowPart;
      ft.dwHighDateTime = uli.HighPart;

      SYSTEMTIME st;
      FileTimeToSystemTime(&ft, &st);

      DATE date;
      SystemTimeToVariantTime(&st, &date);

      ThrowIfFailed(telescope_->put_UTCDate(date));
    });
  }

protected:
  void OnConnected() override {
    // **连接时重新加载能力**
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    capabilities_.loaded = false;
  }

private:
  void LoadCapabilities() const {
    capabilities_.loaded = true;

    try {
      VARIANT_BOOL result;

      ThrowIfFailed(telescope_->get_CanSlew(&result));
      capabilities_.can_slew = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanSlewAsync(&result));
      capabilities_.can_slew_async = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanSync(&result));
      capabilities_.can_sync = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanPark(&result));
      capabilities_.can_park = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanUnpark(&result));
      capabilities_.can_unpark = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanFindHome(&result));
      capabilities_.can_find_home = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanPulseGuide(&result));
      capabilities_.can_pulse_guide = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanSetTracking(&result));
      capabilities_.can_set_tracking = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanSlewAltAz(&result));
      capabilities_.can_slew_altaz = (result == VARIANT_TRUE);

      ThrowIfFailed(telescope_->get_CanSyncAltAz(&result));
      capabilities_.can_sync_altaz = (result == VARIANT_TRUE);
    } catch (const AscomException &) {
      // **如果查询能力失败，假设所有能力都不支持**
      capabilities_ = TelescopeCapabilities{};
      capabilities_.loaded = true;
    }
  }

  template <typename CapabilityFunc>
  void EnsureCapability(CapabilityFunc &&func,
                        const std::string &operation) const {
    if (!func()) {
      throw AscomException(0x80040400,
                           operation + " is not supported by this telescope");
    }
  }

  void WaitForSlewComplete() {
    // **轮询等待slew完成**
    const auto timeout = std::chrono::minutes(10); // **最长等待10分钟**
    const auto poll_interval = std::chrono::milliseconds(100);
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        throw AscomException(0x80040408, "Slew operation timeout");
      }

      try {
        if (!IsSlewing()) {
          is_slewing_ = false;
          slew_condition_.notify_all();
          break;
        }
      } catch (const AscomException &) {
        // **查询状态失败，继续等待**
      }

      std::this_thread::sleep_for(poll_interval);
    }
  }
};