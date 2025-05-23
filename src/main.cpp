#include "AscomCamera.h"
#include "AscomTelescope.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>


// **ASCOM设备发现与管理**
class AscomDeviceManager {
private:
  static std::vector<std::string>
  GetRegisteredProgIDs(const std::string &device_type) {
    std::vector<std::string> prog_ids;

    // **查询注册表中的ASCOM设备**
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

public:
  static std::vector<std::string> GetTelescopeDrivers() {
    return GetRegisteredProgIDs("Telescope");
  }

  static std::vector<std::string> GetCameraDrivers() {
    return GetRegisteredProgIDs("Camera");
  }

  static std::vector<std::string> GetFilterWheelDrivers() {
    return GetRegisteredProgIDs("FilterWheel");
  }

  static std::vector<std::string> GetFocuserDrivers() {
    return GetRegisteredProgIDs("Focuser");
  }
};

// **自动化观测序列**
class ObservingSequence {
private:
  std::unique_ptr<AscomTelescope> telescope_;
  std::unique_ptr<AscomCamera> camera_;
  std::string output_directory_;

public:
  ObservingSequence(const std::string &telescope_prog_id,
                    const std::string &camera_prog_id,
                    const std::string &output_dir)
      : output_directory_(output_dir) {
    try {
      telescope_ = std::make_unique<AscomTelescope>(telescope_prog_id);
      camera_ = std::make_unique<AscomCamera>(camera_prog_id);
    } catch (const AscomException &e) {
      std::cerr << "Failed to create ASCOM devices: " << e.what() << std::endl;
      throw;
    }
  }

  // **设备初始化**
  void Initialize() {
    std::cout << "=== **Initializing ASCOM Devices** ===" << std::endl;

    // **连接望远镜**
    std::cout << "Connecting telescope..." << std::endl;
    telescope_->Connect();
    std::cout << "Telescope: " << telescope_->GetName() << std::endl;
    std::cout << "Version: " << telescope_->GetDriverVersion() << std::endl;

    auto tel_caps = telescope_->GetCapabilities();
    std::cout << "**Telescope capabilities:**" << std::endl;
    std::cout << "  Can Slew: " << (tel_caps.can_slew ? "Yes" : "No")
              << std::endl;
    std::cout << "  Can Track: " << (tel_caps.can_set_tracking ? "Yes" : "No")
              << std::endl;
    std::cout << "  Can Park: " << (tel_caps.can_park ? "Yes" : "No")
              << std::endl;

    // **连接相机**
    std::cout << "\nConnecting camera..." << std::endl;
    camera_->Connect();
    std::cout << "Camera: " << camera_->GetName() << std::endl;
    std::cout << "Version: " << camera_->GetDriverVersion() << std::endl;

    auto cam_caps = camera_->GetCapabilities();
    std::cout << "**Camera capabilities:**" << std::endl;
    std::cout << "  Sensor: " << cam_caps.camera_x_size << "x"
              << cam_caps.camera_y_size << std::endl;
    std::cout << "  Pixel Size: " << cam_caps.pixel_size_x << "x"
              << cam_caps.pixel_size_y << " μm" << std::endl;
    std::cout << "  Can Cool: "
              << (cam_caps.can_set_ccd_temperature ? "Yes" : "No") << std::endl;
    std::cout << "  Max Binning: " << cam_caps.max_bin_x << "x"
              << cam_caps.max_bin_y << std::endl;
  }

  // **目标观测序列**
  void ObserveTarget(const std::string &target_name,
                     const EquatorialCoordinates &coords,
                     const std::vector<ExposureParams> &exposures) {
    std::cout << "\n=== **Observing Target: " << target_name
              << "** ===" << std::endl;

    try {
      // **指向目标**
      std::cout << "Slewing to target coordinates..." << std::endl;
      auto slew_future = telescope_->SlewToCoordinatesAsync(coords);

      // **配置相机**
      CameraConfig cam_config;
      cam_config.bin_x = cam_config.bin_y = 2; // 2x2分档
      cam_config.cooler_on = true;
      cam_config.set_temperature = -20.0;

      std::cout << "Configuring camera..." << std::endl;
      camera_->ApplyConfig(cam_config);

      // **等待望远镜到位**
      std::cout << "Waiting for slew to complete..." << std::endl;
      slew_future.wait();

      // **启用跟踪**
      if (telescope_->GetCapabilities().can_set_tracking) {
        telescope_->SetTracking(true);
        std::cout << "Tracking enabled." << std::endl;
      }

      // **执行曝光序列**
      for (size_t i = 0; i < exposures.size(); ++i) {
        const auto &exp = exposures[i];
        std::cout << "Taking exposure " << (i + 1) << "/" << exposures.size()
                  << " (" << exp.duration << "s)..." << std::endl;

        // **开始曝光**
        auto exp_future = camera_->StartExposureAsync(exp);

        // **监控曝光进度**
        MonitorExposure(exp.duration);

        // **等待曝光完成**
        exp_future.wait();

        // **下载图像**
        std::cout << "Downloading image..." << std::endl;
        auto image = camera_->GetImage<uint16_t>();

        // **保存图像**
        std::string filename = GenerateFilename(target_name, i + 1, exp);
        SaveImage(image, filename);

        // **显示统计信息**
        auto stats = image.CalculateStatistics();
        std::cout << "Image statistics: Min=" << stats.min_value
                  << ", Max=" << stats.max_value << ", Mean=" << std::fixed
                  << std::setprecision(1) << stats.mean
                  << ", StdDev=" << stats.std_dev << std::endl;
      }

      std::cout << "Target observation completed successfully!" << std::endl;
    } catch (const AscomException &e) {
      std::cerr << "**ERROR** during observation: " << e.what() << std::endl;
      throw;
    }
  }

  // **关闭序列**
  void Shutdown() {
    std::cout << "\n=== **Shutting Down** ===" << std::endl;

    try {
      // **停止跟踪**
      if (telescope_->IsConnected() &&
          telescope_->GetCapabilities().can_set_tracking) {
        telescope_->SetTracking(false);
        std::cout << "Tracking disabled." << std::endl;
      }

      // **停放望远镜**
      if (telescope_->IsConnected() && telescope_->GetCapabilities().can_park) {
        std::cout << "Parking telescope..." << std::endl;
        telescope_->Park();
      }

      // **关闭相机制冷**
      if (camera_->IsConnected() &&
          camera_->GetCapabilities().can_set_ccd_temperature) {
        camera_->SetCoolerOn(false);
        std::cout << "Camera cooler turned off." << std::endl;
      }

      // **断开设备连接**
      if (camera_->IsConnected()) {
        camera_->Disconnect();
        std::cout << "Camera disconnected." << std::endl;
      }

      if (telescope_->IsConnected()) {
        telescope_->Disconnect();
        std::cout << "Telescope disconnected." << std::endl;
      }

      std::cout << "Shutdown completed." << std::endl;
    } catch (const AscomException &e) {
      std::cerr << "**ERROR** during shutdown: " << e.what() << std::endl;
    }
  }

  // **获取性能报告**
  void PrintPerformanceReport() const {
    std::cout << "\n=== **Performance Report** ===" << std::endl;

    if (telescope_) {
      std::cout << "**Telescope Operations:**" << std::endl;
      auto tel_metrics = telescope_->GetPerformanceMetrics();
      for (const auto &[operation, duration] : tel_metrics) {
        std::cout << "  " << operation << ": " << duration.count() << " μs"
                  << std::endl;
      }
    }

    if (camera_) {
      std::cout << "**Camera Operations:**" << std::endl;
      auto cam_metrics = camera_->GetPerformanceMetrics();
      for (const auto &[operation, duration] : cam_metrics) {
        std::cout << "  " << operation << ": " << duration.count() << " μs"
                  << std::endl;
      }
    }
  }

private:
  void MonitorExposure(double duration_seconds) {
    auto start_time = std::chrono::steady_clock::now();
    auto total_duration = std::chrono::duration<double>(duration_seconds);

    while (true) {
      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed >= total_duration)
        break;

      auto state = camera_->GetCameraState();
      if (state != CameraState::Exposing)
        break;

      // **显示进度**
      double progress = elapsed.count() / duration_seconds * 100.0;
      std::cout << "\rExposure progress: " << std::fixed << std::setprecision(1)
                << progress << "%" << std::flush;

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << std::endl;
  }

  std::string GenerateFilename(const std::string &target, int sequence,
                               const ExposureParams &exp) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << output_directory_ << "/" << target << "_"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_"
       << std::setfill('0') << std::setw(3) << sequence << "_"
       << static_cast<int>(exp.duration) << "s"
       << (exp.light_frame ? "_light" : "_dark") << ".fits";

    return ss.str();
  }

  template <typename T>
  void SaveImage(const Image<T> &image, const std::string &filename) {
    // **简化的FITS文件保存（实际项目中应使用专业的FITS库）**
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Failed to create output file: " + filename);
    }

    // **写入简单的二进制头部**
    int width = image.GetWidth();
    int height = image.GetHeight();
    int bit_depth = image.GetBitDepth();

    file.write(reinterpret_cast<const char *>(&width), sizeof(width));
    file.write(reinterpret_cast<const char *>(&height), sizeof(height));
    file.write(reinterpret_cast<const char *>(&bit_depth), sizeof(bit_depth));

    // **写入图像数据**
    file.write(reinterpret_cast<const char *>(image.GetData()),
               image.GetPixelCount() * sizeof(T));

    std::cout << "Image saved to: " << filename << std::endl;
  }
};

int main() {
  try {
    // **初始化ASCOM环境**
    AscomDeviceFactory::Initialize();

    std::cout << "=== **ASCOM Device Discovery** ===" << std::endl;

    // **发现可用设备**
    auto telescopes = AscomDeviceManager::GetTelescopeDrivers();
    auto cameras = AscomDeviceManager::GetCameraDrivers();

    std::cout << "**Available Telescopes:**" << std::endl;
    for (const auto &tel : telescopes) {
      std::cout << "  " << tel << std::endl;
    }

    std::cout << "**Available Cameras:**" << std::endl;
    for (const auto &cam : cameras) {
      std::cout << "  " << cam << std::endl;
    }

    if (telescopes.empty() || cameras.empty()) {
      std::cout
          << "**WARNING:** No ASCOM devices found. Using simulator drivers."
          << std::endl;
    }

    // **使用第一个可用设备或模拟器**
    std::string telescope_prog_id =
        telescopes.empty() ? "ASCOM.Simulator.Telescope" : telescopes[0];
    std::string camera_prog_id =
        cameras.empty() ? "ASCOM.Simulator.Camera" : cameras[0];

    // **创建观测序列**
    ObservingSequence sequence(telescope_prog_id, camera_prog_id, "./images");

    // **初始化设备**
    sequence.Initialize();

    // **定义观测目标**
    std::vector<EquatorialCoordinates> targets = {
        {2.0969, 57.0800}, // Andromeda Galaxy (M31)
        {5.5944, -5.3911}, // Orion Nebula (M42)
        {12.4014, 12.5833} // Virgo Galaxy (M87)
    };

    std::vector<std::string> target_names = {"M31_Andromeda", "M42_Orion",
                                             "M87_Virgo"};

    // **定义曝光序列**
    std::vector<ExposureParams> exposures = {
        {60.0, true, "Clear", ""}, // 60秒光帧
        {60.0, true, "Clear", ""}, // 60秒光帧
        {60.0, false, "Clear", ""} // 60秒暗帧
    };

    // **执行观测**
    for (size_t i = 0; i < targets.size(); ++i) {
      if (i >= target_names.size())
        break;

      try {
        sequence.ObserveTarget(target_names[i], targets[i], exposures);

        // **目标之间的休息时间**
        if (i < targets.size() - 1) {
          std::cout << "Waiting 30 seconds before next target..." << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(30));
        }
      } catch (const AscomException &e) {
        std::cerr << "**FAILED** to observe " << target_names[i] << ": "
                  << e.what() << std::endl;
        continue; // **继续下一个目标**
      }
    }

    // **显示性能报告**
    sequence.PrintPerformanceReport();

    // **优雅关闭**
    sequence.Shutdown();

    std::cout << "\n=== **All observations completed successfully!** ==="
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "**FATAL ERROR:** " << e.what() << std::endl;
    return 1;
  }

  // **清理COM环境**
  AscomDeviceFactory::Cleanup();
  return 0;
}