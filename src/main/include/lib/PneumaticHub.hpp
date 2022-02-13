#pragma once

#include <frc/PneumaticHub.h>
#include <frc/SensorUtil.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableRegistry.h>

namespace RJ {

class PneumaticHub : public frc::PneumaticHub,
                      public wpi::Sendable,
                      public wpi::SendableHelper<RJ::PneumaticHub>
{    
public:
    explicit PneumaticHub(int module = frc::SensorUtil::GetDefaultREVPHModule());

    void InitSendable(wpi::SendableBuilder &builder) override;

    typedef union {
        frc::PneumaticHub::Faults bit_faults;
        uint32_t uint32_faults;
    } u_faults;

        typedef union {
        frc::PneumaticHub::StickyFaults bit_stickyfaults;
        uint32_t uint32_stickyfaults;
    } u_stickyfaults;
};

}