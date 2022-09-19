package io.robojackets.config;

import lombok.Builder;
import lombok.Getter;

@Builder
@Getter
public class FeedForwardConfig {
  double kS;
  double kV;
  double kA;
}
