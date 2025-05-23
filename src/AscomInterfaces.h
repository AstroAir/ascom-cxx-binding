#pragma once

#include <vector>
#include <string>
#include <variant>
#include <chrono>
#include <optional>

// clang-format off
#include <windows.h>
#include <winscard.h>
#include <comdef.h>
// clang-format on

// **ASCOM标准接口GUID定义**
const IID IID_IAscomDriver     = {0x76618F90, 0x032F, 0x4424, {0x94, 0xBA, 0x44, 0x51, 0x38, 0x5A, 0xBD, 0x5C}};
const IID IID_ITelescope       = {0x0C88F828, 0x0E09, 0x4C6F, {0xA8, 0x5D, 0x73, 0x1A, 0x8B, 0x86, 0x08, 0x24}};
const IID IID_ICamera          = {0x1B1A1A67, 0x2F1E, 0x49A7, {0x88, 0x1C, 0x68, 0x2A, 0xF0, 0x5A, 0xD9, 0x36}};
const IID IID_IFilterWheel     = {0x2E9F2F6A, 0x7D4E, 0x4D8B, {0xA9, 0x2C, 0x79, 0x3B, 0x16, 0xBF, 0xE8, 0x47}};
const IID IID_IFocuser         = {0x3FA0A3DB, 0x8E5F, 0x4E9C, {0xBA, 0x3D, 0x8A, 0x4C, 0x27, 0xCF, 0xF9, 0x58}};

// **ASCOM设备状态枚举**
enum class AscomDeviceState : int
{
    Unknown = -1,
    Disconnected = 0,
    Connected = 1,
    Error = 2
};

// **望远镜轴状态**
enum class TelescopeAxisState : int
{
    Idle = 0,
    Slewing = 1,
    Tracking = 2,
    Parked = 3,
    Error = 4
};

// **相机状态**
enum class CameraState : int
{
    Idle = 0,
    Waiting = 1,
    Exposing = 2,
    Reading = 3,
    Download = 4,
    Error = 5
};

// **ASCOM异常类型**
class AscomException : public std::runtime_error
{
private:
    int error_code_;
    std::string source_;
    
public:
    AscomException(int code, const std::string& message, const std::string& source = "")
        : std::runtime_error(message), error_code_(code), source_(source)
    {
    }
    
    int GetErrorCode() const noexcept { return error_code_; }
    const std::string& GetSource() const noexcept { return source_; }
};

// **ASCOM数据类型映射**
using AscomVariant = std::variant<int, double, bool, std::string, std::vector<int>, std::vector<double>>;

// **坐标结构体**
struct EquatorialCoordinates
{
    double right_ascension;  // 赤经 (小时)
    double declination;      // 赤纬 (度)
    
    EquatorialCoordinates(double ra = 0.0, double dec = 0.0)
        : right_ascension(ra), declination(dec) {}
};

struct AltAzCoordinates
{
    double altitude;   // 高度角 (度)
    double azimuth;    // 方位角 (度)
    
    AltAzCoordinates(double alt = 0.0, double az = 0.0)
        : altitude(alt), azimuth(az) {}
};