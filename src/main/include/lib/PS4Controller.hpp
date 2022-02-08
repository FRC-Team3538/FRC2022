#pragma once

#include <frc/PS4Controller.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>
#include <wpi/sendable/SendableRegistry.h>

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