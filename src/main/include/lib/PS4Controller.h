#pragma once

#include <frc/PS4Controller.h>            // for PS4Controller
#include <wpi/sendable/Sendable.h>        // for Sendable
#include <wpi/sendable/SendableHelper.h>  // for SendableHelper
namespace wpi { class SendableBuilder; }

namespace RJ {

class PS4Controller : public frc::PS4Controller,
                      public wpi::Sendable,
                      public wpi::SendableHelper<RJ::PS4Controller>
{
public:
    explicit PS4Controller(int port);

    void InitSendable(wpi::SendableBuilder &builder) override;
};

}