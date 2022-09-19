package io.robojackets.lib;

public class FalconUtils {
  public static double NativeVelocityToRPS(double nativeVelocity) {
    return nativeVelocity / 2048 * 10;
  }

  public static double NativeVelocityToRPM(double nativeVelocity) {
    return NativeVelocityToRPS(nativeVelocity) * 60;
  }
}
