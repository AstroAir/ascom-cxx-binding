#pragma once
#include "AscomDriver.h"
#include <array>
#include <memory>

// **简化的ASCOM相机接口定义**
MIDL_INTERFACE("1B1A1A67-2F1E-49A7-881C-682AF05AD936")
ICamera : public IAscomDriver {
public:
  virtual HRESULT STDMETHODCALLTYPE AbortExposure() = 0;
  virtual HRESULT STDMETHODCALLTYPE get_BinX(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_BinX(SHORT newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_BinY(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_BinY(SHORT newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CameraState(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CameraXSize(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CameraYSize(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanAbortExposure(VARIANT_BOOL *
                                                         pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanAsymmetricBin(VARIANT_BOOL *
                                                         pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanGetCoolerPower(VARIANT_BOOL *
                                                          pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanPulseGuide(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanSetCCDTemperature(VARIANT_BOOL *
                                                             pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CanStopExposure(VARIANT_BOOL *
                                                        pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CCDTemperature(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CoolerOn(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_CoolerOn(VARIANT_BOOL newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_CoolerPower(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ElectronsPerADU(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ExposureMax(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ExposureMin(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ExposureResolution(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_FastReadout(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_FastReadout(VARIANT_BOOL newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_FullWellCapacity(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Gain(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_Gain(SHORT newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_Gains(SAFEARRAY * *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_HasShutter(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_HeatSinkTemperature(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ImageArray(SAFEARRAY * *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ImageArrayVariant(SAFEARRAY *
                                                          *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ImageReady(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_IsPulseGuiding(VARIANT_BOOL * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_LastExposureDuration(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_LastExposureStartTime(BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_MaxADU(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_MaxBinX(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_MaxBinY(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_NumX(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_NumX(long newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_NumY(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_NumY(long newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_PixelSizeX(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_PixelSizeY(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE PulseGuide(SHORT Direction,
                                               long Duration) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ReadoutMode(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_ReadoutMode(SHORT newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_ReadoutModes(SAFEARRAY * *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_SensorName(BSTR * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_SensorType(SHORT * pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_SetCCDTemperature(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_SetCCDTemperature(double newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE StartExposure(double Duration,
                                                  VARIANT_BOOL Light) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_StartX(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_StartX(long newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE get_StartY(long *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_StartY(long newVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE StopExposure() = 0;
  virtual HRESULT STDMETHODCALLTYPE get_SubExposureDuration(double *pVal) = 0;
  virtual HRESULT STDMETHODCALLTYPE put_SubExposureDuration(double newVal) = 0;
};

// **图像数据结构**
template <typename T> class Image {
private:
  std::unique_ptr<T[]> data_;
  int width_;
  int height_;
  int bit_depth_;

public:
  Image(int width, int height, int bit_depth)
      : width_(width), height_(height), bit_depth_(bit_depth),
        data_(std::make_unique<T[]>(width * height)) {}

  // **移动构造**
  Image(Image &&other) noexcept
      : data_(std::move(other.data_)), width_(other.width_),
        height_(other.height_), bit_depth_(other.bit_depth_) {
    other.width_ = other.height_ = other.bit_depth_ = 0;
  }

  Image &operator=(Image &&other) noexcept {
    if (this != &other) {
      data_ = std::move(other.data_);
      width_ = other.width_;
      height_ = other.height_;
      bit_depth_ = other.bit_depth_;
      other.width_ = other.height_ = other.bit_depth_ = 0;
    }
    return *this;
  }

  // **禁用拷贝**
  Image(const Image &) = delete;
  Image &operator=(const Image &) = delete;

  // **访问器**
  int GetWidth() const noexcept { return width_; }
  int GetHeight() const noexcept { return height_; }
  int GetBitDepth() const noexcept { return bit_depth_; }
  size_t GetPixelCount() const noexcept {
    return static_cast<size_t>(width_) * height_;
  }

  const T *GetData() const noexcept { return data_.get(); }
  T *GetData() noexcept { return data_.get(); }

  T GetPixel(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
      throw std::out_of_range("Pixel coordinates out of range");
    return data_[y * width_ + x];
  }

  void SetPixel(int x, int y, T value) {
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
      throw std::out_of_range("Pixel coordinates out of range");
    data_[y * width_ + x] = value;
  }

  // **统计信息**
  struct Statistics {
    T min_value;
    T max_value;
    double mean;
    double std_dev;
  };

  Statistics CalculateStatistics() const {
    if (GetPixelCount() == 0)
      return {T{}, T{}, 0.0, 0.0};

    T min_val = data_[0];
    T max_val = data_[0];
    double sum = 0.0;

    for (size_t i = 0; i < GetPixelCount(); ++i) {
      T val = data_[i];
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
      sum += static_cast<double>(val);
    }

    double mean = sum / GetPixelCount();
    double variance = 0.0;

    for (size_t i = 0; i < GetPixelCount(); ++i) {
      double diff = static_cast<double>(data_[i]) - mean;
      variance += diff * diff;
    }

    double std_dev = std::sqrt(variance / GetPixelCount());

    return {min_val, max_val, mean, std_dev};
  }
};

// **相机配置结构**
struct CameraConfig {
  int start_x = 0;
  int start_y = 0;
  int width = 0;  // 0表示使用全尺寸
  int height = 0; // 0表示使用全尺寸
  int bin_x = 1;
  int bin_y = 1;
  int gain = 0;
  double set_temperature = -10.0;
  bool cooler_on = false;
  bool fast_readout = false;
  int readout_mode = 0;
};

// **曝光参数结构**
struct ExposureParams {
  double duration;         // 曝光时间（秒）
  bool light_frame = true; // true=light frame, false=dark frame
  std::string filter;      // 滤镜名称（可选）
  std::string object_name; // 目标名称（可选）
};

// **ASCOM相机驱动封装**
class AscomCamera : public AscomDriverBase {
private:
  AscomPtr<ICamera> camera_;

  // **相机能力缓存**
  struct CameraCapabilities {
    bool can_abort_exposure = false;
    bool can_asymmetric_bin = false;
    bool can_get_cooler_power = false;
    bool can_pulse_guide = false;
    bool can_set_ccd_temperature = false;
    bool can_stop_exposure = false;
    bool has_shutter = false;
    bool loaded = false;

    // **硬件参数**
    int camera_x_size = 0;
    int camera_y_size = 0;
    double pixel_size_x = 0.0;
    double pixel_size_y = 0.0;
    int max_bin_x = 1;
    int max_bin_y = 1;
    double exposure_min = 0.0;
    double exposure_max = 3600.0;
    long max_adu = 65535;

    std::vector<std::string> readout_modes;
    std::vector<int> gains;

  } capabilities_;

  mutable std::mutex capabilities_mutex_;

  // **异步曝光支持**
  std::atomic<CameraState> exposure_state_{CameraState::Idle};
  std::mutex exposure_mutex_;
  std::condition_variable exposure_condition_;
  std::future<void> exposure_future_;

public:
  explicit AscomCamera(const std::string &prog_id) : AscomDriverBase(prog_id) {
    camera_ = AscomDeviceFactory::CreateDevice<ICamera>(prog_id);
  }

  ~AscomCamera() {
    try {
      if (exposure_state_ == CameraState::Exposing ||
          exposure_state_ == CameraState::Reading) {
        AbortExposure();
      }
    } catch (...) {
      // **析构函数中不抛异常**
    }
  }

  // **相机能力查询**
  const CameraCapabilities &GetCapabilities() const {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);

    if (!capabilities_.loaded) {
      LoadCapabilities();
    }

    return capabilities_;
  }

  // **相机配置管理**
  void ApplyConfig(const CameraConfig &config) {
    MeasurePerformance("ApplyConfig", [&]() {
      auto caps = GetCapabilities();

      // **验证配置有效性**
      if (config.width > 0 && config.height > 0) {
        if (config.start_x + config.width > caps.camera_x_size ||
            config.start_y + config.height > caps.camera_y_size) {
          throw AscomException(0x80040005, "Invalid subframe configuration");
        }

        ThrowIfFailed(camera_->put_StartX(config.start_x));
        ThrowIfFailed(camera_->put_StartY(config.start_y));
        ThrowIfFailed(camera_->put_NumX(config.width));
        ThrowIfFailed(camera_->put_NumY(config.height));
      }

      // **设置Binning**
      if (config.bin_x <= caps.max_bin_x && config.bin_y <= caps.max_bin_y) {
        if (!caps.can_asymmetric_bin && config.bin_x != config.bin_y) {
          throw AscomException(0x80040005, "Asymmetric binning not supported");
        }

        ThrowIfFailed(camera_->put_BinX(config.bin_x));
        ThrowIfFailed(camera_->put_BinY(config.bin_y));
      }

      // **设置增益**
      if (!caps.gains.empty()) {
        auto it = std::find(caps.gains.begin(), caps.gains.end(), config.gain);
        if (it != caps.gains.end()) {
          ThrowIfFailed(camera_->put_Gain(config.gain));
        }
      }

      // **设置读出模式**
      if (config.readout_mode < caps.readout_modes.size()) {
        ThrowIfFailed(camera_->put_ReadoutMode(config.readout_mode));
      }

      // **设置快速读出**
      ThrowIfFailed(camera_->put_FastReadout(
          config.fast_readout ? VARIANT_TRUE : VARIANT_FALSE));

      // **设置制冷**
      if (caps.can_set_ccd_temperature) {
        ThrowIfFailed(camera_->put_SetCCDTemperature(config.set_temperature));
        ThrowIfFailed(camera_->put_CoolerOn(config.cooler_on ? VARIANT_TRUE
                                                             : VARIANT_FALSE));
      }
    });
  }

  CameraConfig GetCurrentConfig() const {
    return MeasurePerformance("GetCurrentConfig", [this]() {
      CameraConfig config;

      long value;
      SHORT short_value;
      VARIANT_BOOL bool_value;
      double double_value;

      ThrowIfFailed(camera_->get_StartX(&value));
      config.start_x = value;

      ThrowIfFailed(camera_->get_StartY(&value));
      config.start_y = value;

      ThrowIfFailed(camera_->get_NumX(&value));
      config.width = value;

      ThrowIfFailed(camera_->get_NumY(&value));
      config.height = value;

      ThrowIfFailed(camera_->get_BinX(&short_value));
      config.bin_x = short_value;

      ThrowIfFailed(camera_->get_BinY(&short_value));
      config.bin_y = short_value;

      ThrowIfFailed(camera_->get_Gain(&short_value));
      config.gain = short_value;

      if (GetCapabilities().can_set_ccd_temperature) {
        ThrowIfFailed(camera_->get_SetCCDTemperature(&double_value));
        config.set_temperature = double_value;

        ThrowIfFailed(camera_->get_CoolerOn(&bool_value));
        config.cooler_on = (bool_value == VARIANT_TRUE);
      }

      ThrowIfFailed(camera_->get_FastReadout(&bool_value));
      config.fast_readout = (bool_value == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_ReadoutMode(&short_value));
      config.readout_mode = short_value;

      return config;
    });
  }

  // **曝光控制**
  std::future<void> StartExposureAsync(const ExposureParams &params) {
    std::unique_lock<std::mutex> lock(exposure_mutex_);

    if (exposure_state_ != CameraState::Idle) {
      throw AscomException(0x80040006, "Camera is not idle");
    }

    // **验证曝光参数**
    auto caps = GetCapabilities();
    if (params.duration < caps.exposure_min ||
        params.duration > caps.exposure_max) {
      throw AscomException(0x80040005, "Invalid exposure duration");
    }

    exposure_state_ = CameraState::Exposing;

    exposure_future_ = std::async(std::launch::async, [this, params]() {
      try {
        MeasurePerformance("StartExposure", [&]() {
          ThrowIfFailed(camera_->StartExposure(
              params.duration,
              params.light_frame ? VARIANT_TRUE : VARIANT_FALSE));
        });

        // **等待曝光完成**
        WaitForExposureComplete();
      } catch (...) {
        exposure_state_ = CameraState::Error;
        exposure_condition_.notify_all();
        throw;
      }
    });

    return std::future<void>(exposure_future_.get());
  }

  void StartExposure(const ExposureParams &params) {
    auto future = StartExposureAsync(params);
    future.wait(); // **同步等待完成**
  }

  void AbortExposure() {
    if (!GetCapabilities().can_abort_exposure) {
      throw AscomException(0x80040400, "Abort exposure not supported");
    }

    std::unique_lock<std::mutex> lock(exposure_mutex_);

    if (exposure_state_ != CameraState::Exposing &&
        exposure_state_ != CameraState::Reading) {
      return;
    }

    MeasurePerformance("AbortExposure",
                       [this]() { ThrowIfFailed(camera_->AbortExposure()); });

    exposure_state_ = CameraState::Idle;
    exposure_condition_.notify_all();
  }

  void StopExposure() {
    if (!GetCapabilities().can_stop_exposure) {
      throw AscomException(0x80040400, "Stop exposure not supported");
    }

    MeasurePerformance("StopExposure",
                       [this]() { ThrowIfFailed(camera_->StopExposure()); });
  }

  // **图像获取**
  template <typename T = uint16_t> Image<T> GetImage() {
    std::unique_lock<std::mutex> lock(exposure_mutex_);

    if (exposure_state_ != CameraState::Download) {
      throw AscomException(0x80040006, "No image available for download");
    }

    return MeasurePerformance("GetImage", [this]() -> Image<T> {
      // **检查图像是否就绪**
      VARIANT_BOOL ready;
      ThrowIfFailed(camera_->get_ImageReady(&ready));
      if (ready != VARIANT_TRUE) {
        throw AscomException(0x80040006, "Image not ready");
      }

      // **获取当前配置**
      auto config = GetCurrentConfig();
      int width = config.width;
      int height = config.height;

      if (width == 0 || height == 0) {
        auto caps = GetCapabilities();
        width = caps.camera_x_size / config.bin_x;
        height = caps.camera_y_size / config.bin_y;
      } else {
        width /= config.bin_x;
        height /= config.bin_y;
      }

      // **获取图像数据**
      SAFEARRAY *image_array = nullptr;
      ThrowIfFailed(camera_->get_ImageArray(&image_array));

      if (!image_array) {
        throw AscomException(0x80040006, "Failed to get image array");
      }

      // **创建图像对象并拷贝数据**
      Image<T> image(width, height, sizeof(T) * 8);

      try {
        // **访问SAFEARRAY数据**
        void *array_data;
        ThrowIfFailed(SafeArrayAccessData(image_array, &array_data));

        // **根据数据类型进行转换**
        VARTYPE array_type;
        ThrowIfFailed(SafeArrayGetVartype(image_array, &array_type));

        T *dest = image.GetData();
        size_t pixel_count = image.GetPixelCount();

        switch (array_type) {
        case VT_I2: // 16-bit signed
        {
          auto *src = static_cast<int16_t *>(array_data);
          for (size_t i = 0; i < pixel_count; ++i) {
            dest[i] = static_cast<T>(std::max(0, static_cast<int>(src[i])));
          }
          break;
        }
        case VT_UI2: // 16-bit unsigned
        {
          auto *src = static_cast<uint16_t *>(array_data);
          std::copy(src, src + pixel_count, dest);
          break;
        }
        case VT_I4: // 32-bit signed
        {
          auto *src = static_cast<int32_t *>(array_data);
          for (size_t i = 0; i < pixel_count; ++i) {
            dest[i] = static_cast<T>(std::max(0, src[i]));
          }
          break;
        }
        case VT_R8: // Double
        {
          auto *src = static_cast<double *>(array_data);
          for (size_t i = 0; i < pixel_count; ++i) {
            dest[i] = static_cast<T>(std::max(0.0, src[i]));
          }
          break;
        }
        default:
          throw AscomException(0x80040006, "Unsupported image array type");
        }

        ThrowIfFailed(SafeArrayUnaccessData(image_array));
        SafeArrayDestroy(image_array);

        exposure_state_ = CameraState::Idle;
        return image;
      } catch (...) {
        SafeArrayUnaccessData(image_array);
        SafeArrayDestroy(image_array);
        throw;
      }
    });
  }

  // **状态查询**
  CameraState GetCameraState() const {
    if (exposure_state_ != CameraState::Idle) {
      return exposure_state_;
    }

    return MeasurePerformance("GetCameraState", [this]() {
      SHORT state;
      ThrowIfFailed(camera_->get_CameraState(&state));
      return static_cast<CameraState>(state);
    });
  }

  bool IsImageReady() const {
    return MeasurePerformance("IsImageReady", [this]() {
      VARIANT_BOOL ready;
      ThrowIfFailed(camera_->get_ImageReady(&ready));
      return ready == VARIANT_TRUE;
    });
  }

  // **温度控制**
  double GetCCDTemperature() const {
    return MeasurePerformance("GetCCDTemperature", [this]() {
      double temp;
      ThrowIfFailed(camera_->get_CCDTemperature(&temp));
      return temp;
    });
  }

  double GetHeatSinkTemperature() const {
    return MeasurePerformance("GetHeatSinkTemperature", [this]() {
      double temp;
      ThrowIfFailed(camera_->get_HeatSinkTemperature(&temp));
      return temp;
    });
  }

  void SetCoolerOn(bool on) {
    if (!GetCapabilities().can_set_ccd_temperature) {
      throw AscomException(0x80040400, "Temperature control not supported");
    }

    MeasurePerformance("SetCoolerOn", [&]() {
      ThrowIfFailed(camera_->put_CoolerOn(on ? VARIANT_TRUE : VARIANT_FALSE));
    });
  }

  bool IsCoolerOn() const {
    return MeasurePerformance("IsCoolerOn", [this]() {
      VARIANT_BOOL on;
      ThrowIfFailed(camera_->get_CoolerOn(&on));
      return on == VARIANT_TRUE;
    });
  }

  double GetCoolerPower() const {
    if (!GetCapabilities().can_get_cooler_power) {
      throw AscomException(0x80040400, "Cooler power not available");
    }

    return MeasurePerformance("GetCoolerPower", [this]() {
      double power;
      ThrowIfFailed(camera_->get_CoolerPower(&power));
      return power;
    });
  }

  // **导星功能**
  void PulseGuide(GuideDirection direction,
                  std::chrono::milliseconds duration) {
    if (!GetCapabilities().can_pulse_guide) {
      throw AscomException(0x80040400, "Pulse guiding not supported");
    }

    MeasurePerformance("PulseGuide", [&]() {
      ThrowIfFailed(camera_->PulseGuide(static_cast<SHORT>(direction),
                                        static_cast<long>(duration.count())));
    });
  }

  bool IsPulseGuiding() const {
    return MeasurePerformance("IsPulseGuiding", [this]() {
      VARIANT_BOOL guiding;
      ThrowIfFailed(camera_->get_IsPulseGuiding(&guiding));
      return guiding == VARIANT_TRUE;
    });
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
      SHORT short_result;
      double double_result;

      // **能力查询**
      ThrowIfFailed(camera_->get_CanAbortExposure(&bool_result));
      capabilities_.can_abort_exposure = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_CanAsymmetricBin(&bool_result));
      capabilities_.can_asymmetric_bin = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_CanGetCoolerPower(&bool_result));
      capabilities_.can_get_cooler_power = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_CanPulseGuide(&bool_result));
      capabilities_.can_pulse_guide = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_CanSetCCDTemperature(&bool_result));
      capabilities_.can_set_ccd_temperature = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_CanStopExposure(&bool_result));
      capabilities_.can_stop_exposure = (bool_result == VARIANT_TRUE);

      ThrowIfFailed(camera_->get_HasShutter(&bool_result));
      capabilities_.has_shutter = (bool_result == VARIANT_TRUE);

      // **硬件参数**
      ThrowIfFailed(camera_->get_CameraXSize(&long_result));
      capabilities_.camera_x_size = long_result;

      ThrowIfFailed(camera_->get_CameraYSize(&long_result));
      capabilities_.camera_y_size = long_result;

      ThrowIfFailed(camera_->get_PixelSizeX(&double_result));
      capabilities_.pixel_size_x = double_result;

      ThrowIfFailed(camera_->get_PixelSizeY(&double_result));
      capabilities_.pixel_size_y = double_result;

      ThrowIfFailed(camera_->get_MaxBinX(&short_result));
      capabilities_.max_bin_x = short_result;

      ThrowIfFailed(camera_->get_MaxBinY(&short_result));
      capabilities_.max_bin_y = short_result;

      ThrowIfFailed(camera_->get_ExposureMin(&double_result));
      capabilities_.exposure_min = double_result;

      ThrowIfFailed(camera_->get_ExposureMax(&double_result));
      capabilities_.exposure_max = double_result;

      ThrowIfFailed(camera_->get_MaxADU(&long_result));
      capabilities_.max_adu = long_result;

      // **读出模式和增益**
      LoadReadoutModes();
      LoadGains();
    } catch (const AscomException &) {
      capabilities_ = CameraCapabilities{};
      capabilities_.loaded = true;
    }
  }

  void LoadReadoutModes() const {
    try {
      SAFEARRAY *modes_array = nullptr;
      ThrowIfFailed(camera_->get_ReadoutModes(&modes_array));

      if (modes_array) {
        capabilities_.readout_modes = SafeArrayToStringVector(modes_array);
        SafeArrayDestroy(modes_array);
      }
    } catch (...) {
      capabilities_.readout_modes.clear();
    }
  }

  void LoadGains() const {
    try {
      SAFEARRAY *gains_array = nullptr;
      ThrowIfFailed(camera_->get_Gains(&gains_array));

      if (gains_array) {
        capabilities_.gains = SafeArrayToIntVector(gains_array);
        SafeArrayDestroy(gains_array);
      }
    } catch (...) {
      capabilities_.gains.clear();
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

  void WaitForExposureComplete() {
    const auto poll_interval = std::chrono::milliseconds(100);
    const auto timeout = std::chrono::minutes(30); // **最长等待30分钟**
    auto start_time = std::chrono::steady_clock::now();

    while (exposure_state_ == CameraState::Exposing) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        exposure_state_ = CameraState::Error;
        throw AscomException(0x80040408, "Exposure timeout");
      }

      try {
        CameraState current_state = GetCameraState();
        if (current_state == CameraState::Download ||
            current_state == CameraState::Idle) {
          exposure_state_ = CameraState::Download;
          exposure_condition_.notify_all();
          break;
        } else if (current_state == CameraState::Error) {
          exposure_state_ = CameraState::Error;
          exposure_condition_.notify_all();
          throw AscomException(0x80040006, "Camera error during exposure");
        }
      } catch (const AscomException &) {
        // **查询状态失败，继续等待**
      }

      std::this_thread::sleep_for(poll_interval);
    }
  }
};