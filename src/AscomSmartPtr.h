#pragma once
#include "AscomInterfaces.h"
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>


// **ASCOM专用智能指针**
template <typename T> class AscomPtr {
private:
  T *ptr_ = nullptr;
  std::shared_ptr<std::atomic<bool>> valid_flag_;
  mutable std::shared_mutex access_mutex_;

public:
  AscomPtr() : valid_flag_(std::make_shared<std::atomic<bool>>(false)) {}

  explicit AscomPtr(T *p)
      : ptr_(p),
        valid_flag_(std::make_shared<std::atomic<bool>>(p != nullptr)) {
    if (ptr_)
      ptr_->AddRef();
  }

  // **拷贝构造**
  AscomPtr(const AscomPtr &other) : valid_flag_(other.valid_flag_) {
    std::shared_lock<std::shared_mutex> lock(other.access_mutex_);
    ptr_ = other.ptr_;
    if (ptr_ && valid_flag_->load())
      ptr_->AddRef();
  }

  // **拷贝赋值**
  AscomPtr &operator=(const AscomPtr &other) {
    if (this != &other) {
      std::unique_lock<std::shared_mutex> lock1(access_mutex_, std::defer_lock);
      std::shared_lock<std::shared_mutex> lock2(other.access_mutex_,
                                                std::defer_lock);
      std::lock(lock1, lock2);

      Reset();
      ptr_ = other.ptr_;
      valid_flag_ = other.valid_flag_;
      if (ptr_ && valid_flag_->load())
        ptr_->AddRef();
    }
    return *this;
  }

  // **移动语义**
  AscomPtr(AscomPtr &&other) noexcept
      : ptr_(other.ptr_), valid_flag_(std::move(other.valid_flag_)) {
    other.ptr_ = nullptr;
  }

  AscomPtr &operator=(AscomPtr &&other) noexcept {
    if (this != &other) {
      std::unique_lock<std::shared_mutex> lock(access_mutex_);
      Reset();
      ptr_ = other.ptr_;
      valid_flag_ = std::move(other.valid_flag_);
      other.ptr_ = nullptr;
    }
    return *this;
  }

  ~AscomPtr() { Reset(); }

  void Reset(T *p = nullptr) {
    std::unique_lock<std::shared_mutex> lock(access_mutex_);
    if (ptr_ && valid_flag_->load()) {
      ptr_->Release();
    }
    ptr_ = p;
    if (p) {
      p->AddRef();
      valid_flag_->store(true);
    } else {
      valid_flag_->store(false);
    }
  }

  T *Get() const noexcept {
    std::shared_lock<std::shared_mutex> lock(access_mutex_);
    return (valid_flag_->load()) ? ptr_ : nullptr;
  }

  T **GetAddressOf() noexcept {
    std::unique_lock<std::shared_mutex> lock(access_mutex_);
    Reset();
    return &ptr_;
  }

  T *operator->() const {
    auto p = Get();
    if (!p)
      throw AscomException(0x80040204, "Interface pointer is null");
    return p;
  }

  T &operator*() const {
    auto p = Get();
    if (!p)
      throw AscomException(0x80040204, "Interface pointer is null");
    return *p;
  }

  explicit operator bool() const noexcept { return Get() != nullptr; }

  bool IsValid() const noexcept {
    return valid_flag_->load() && Get() != nullptr;
  }

  // **安全调用包装器**
  template <typename Func, typename... Args>
  auto SafeCall(Func &&func, Args &&...args) const
      -> std::invoke_result_t<Func, T *, Args...> {
    std::shared_lock<std::shared_mutex> lock(access_mutex_);
    if (!ptr_ || !valid_flag_->load()) {
      throw AscomException(0x80040204,
                           "Device not connected or interface invalid");
    }

    try {
      return std::invoke(std::forward<Func>(func), ptr_,
                         std::forward<Args>(args)...);
    } catch (const _com_error &e) {
      throw AscomException(e.Error(), e.ErrorMessage(), "COM Error");
    }
  }
};

// **ASCOM设备工厂**
class AscomDeviceFactory {
private:
  static inline std::mutex registry_mutex_;
  static inline bool com_initialized_ = false;

public:
  // **初始化ASCOM环境**
  static void Initialize() {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    if (!com_initialized_) {
      HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
      if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) {
        throw AscomException(hr, "Failed to initialize COM");
      }
      com_initialized_ = true;
    }
  }

  // **清理ASCOM环境**
  static void Cleanup() {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    if (com_initialized_) {
      CoUninitialize();
      com_initialized_ = false;
    }
  }

  // **创建设备实例**
  template <typename T>
  static AscomPtr<T> CreateDevice(const std::string &prog_id) {
    Initialize();

    CLSID clsid;
    std::wstring wide_prog_id = StringToWString(prog_id);

    HRESULT hr = CLSIDFromProgID(wide_prog_id.c_str(), &clsid);
    if (FAILED(hr)) {
      throw AscomException(hr, "Failed to get CLSID for ProgID: " + prog_id);
    }

    T *ptr = nullptr;
    hr = CoCreateInstance(clsid, nullptr,
                          CLSCTX_INPROC_SERVER | CLSCTX_LOCAL_SERVER,
                          __uuidof(T), reinterpret_cast<void **>(&ptr));
    if (FAILED(hr)) {
      throw AscomException(hr, "Failed to create instance of " + prog_id);
    }

    return AscomPtr<T>(ptr);
  }

private:
  static std::wstring StringToWString(const std::string &str) {
    int size = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, nullptr, 0);
    std::wstring result(size, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, str.c_str(), -1, &result[0], size);
    result.pop_back(); // 移除null terminator
    return result;
  }
};