package io.robojackets.config;

import lombok.Builder;
import lombok.Getter;

@Builder
@Getter
public class PIDConfig {
  double kP;
  double kI;
  double kD;
}
