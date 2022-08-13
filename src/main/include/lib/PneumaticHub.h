#pragma once

#include <frc/PneumaticHub.h>             // for PneumaticHub, PneumaticHub:...
#include <frc/SensorUtil.h>               // for SensorUtil
#include <stdint.h>                       // for uint32_t
#include <wpi/sendable/Sendable.h>        // for Sendable
#include <wpi/sendable/SendableHelper.h>  // for SendableHelper
namespace wpi { class SendableBuilder; }

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