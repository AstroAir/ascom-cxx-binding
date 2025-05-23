#pragma once
#include "AscomDriver.h"

// **ASCOM旋转器接口**
MIDL_INTERFACE("5C8A8F3E-B9D4-4F2A-A7E6-93B47E1C8D92")
IRotator : public IAscomDriver
{
public:
    virtual HRESULT STDMETHODCALLTYPE get_CanReverse(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE Halt() = 0;
    virtual HRESULT STDMETHODCALLTYPE get_IsMoving(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE Move(float Position) = 0;
    virtual HRESULT STDMETHODCALLTYPE MoveAbsolute(float Position) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Position(float* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_Reverse(VARIANT_BOOL* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_Reverse(VARIANT_BOOL newVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_StepSize(float* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE get_TargetPosition(float* pVal) = 0;
    virtual HRESULT STDMETHODCALLTYPE put_TargetPosition(float newVal) = 0;
};

// **旋转器驱动封装**
class AscomRotator : public AscomDriverBase
{
private:
    AscomPtr<IRotator> rotator_;
    std::atomic<bool> movement_abort_requested_{false};
    mutable std::mutex movement_mutex_;
    
public:
    explicit AscomRotator(const std::string& prog_id)
        : AscomDriverBase(prog_id)
    {
        rotator_ = AscomDeviceFactory::CreateDevice<IRotator>(prog_id);
    }
    
    // **位置控制**
    float GetPosition() const
    {
        return MeasurePerformance("GetPosition", [this]() {
            float position;
            ThrowIfFailed(rotator_->get_Position(&position));
            return position;
        });
    }
    
    void MoveTo(float angle)
    {
        // **规范化角度到0-360度**
        angle = std::fmod(angle, 360.0f);
        if (angle < 0) angle += 360.0f;
        
        std::unique_lock<std::mutex> lock(movement_mutex_);
        movement_abort_requested_ = false;
        
        try
        {
            MeasurePerformance("MoveTo", [&]() {
                ThrowIfFailed(rotator_->MoveAbsolute(angle));
            });
            
            WaitForMovementComplete();
        }
        catch (...)
        {
            movement_abort_requested_ = false;
            throw;
        }
    }
    
    std::future<void> MoveToAsync(float angle)
    {
        return std::async(std::launch::async, [this, angle]() {
            MoveTo(angle);
        });
    }
    
    void MoveBy(float degrees)
    {
        float current = GetPosition();
        MoveTo(current + degrees);
    }
    
    void Halt()
    {
        MeasurePerformance("Halt", [this]() {
            ThrowIfFailed(rotator_->Halt());
        });
        
        movement_abort_requested_ = true;
    }
    
    bool IsMoving() const
    {
        return MeasurePerformance("IsMoving", [this]() {
            VARIANT_BOOL moving;
            ThrowIfFailed(rotator_->get_IsMoving(&moving));
            return moving == VARIANT_TRUE;
        });
    }
    
    // **反向控制**
    bool GetReverse() const
    {
        return MeasurePerformance("GetReverse", [this]() {
            VARIANT_BOOL reverse;
            ThrowIfFailed(rotator_->get_Reverse(&reverse));
            return reverse == VARIANT_TRUE;
        });
    }
    
    void SetReverse(bool reverse)
    {
        MeasurePerformance("SetReverse", [&]() {
            ThrowIfFailed(rotator_->put_Reverse(reverse ? VARIANT_TRUE : VARIANT_FALSE));
        });
    }
    
    float GetStepSize() const
    {
        return MeasurePerformance("GetStepSize", [this]() {
            float step_size;
            ThrowIfFailed(rotator_->get_StepSize(&step_size));
            return step_size;
        });
    }
    
private:
    void WaitForMovementComplete()
    {
        const auto poll_interval = std::chrono::milliseconds(100);
        const auto timeout = std::chrono::minutes(2);
        auto start_time = std::chrono::steady_clock::now();
        
        while (!movement_abort_requested_)
        {
            if (std::chrono::steady_clock::now() - start_time > timeout)
            {
                throw AscomException(0x80040408, "Rotator movement timeout");
            }
            
            try
            {
                if (!IsMoving())
                {
                    break;
                }
            }
            catch (...) {}
            
            std::this_thread::sleep_for(poll_interval);
        }
        
        movement_abort_requested_ = false;
    }
};