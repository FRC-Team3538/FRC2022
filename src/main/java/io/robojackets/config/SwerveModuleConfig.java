package io.robojackets.config;

import lombok.Builder;
import lombok.Getter;

@Builder
@Getter
public class SwerveModuleConfig {
  double angleOffsetRadians;

  double maxTurnVelocityRadiansPerSecond;
  double maxTurnAccelRadiansPerSecondPerSecond;

  PIDConfig drivePIDConfig;
  FeedForwardConfig driveFeedForwardConfig;

  PIDConfig turnPIDConfig;
  FeedForwardConfig turnFeedForwardConfig;
}
