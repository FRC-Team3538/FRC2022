package io.robojackets.lib;

public class UnitConversion {
  public static final double RADIANS_PER_ROTATION = Math.PI * 2.0;
  public static final double DEGREES_PER_ROTATION = 360.0;
  public static final double DEGREES_PER_RADIAN = DEGREES_PER_ROTATION / RADIANS_PER_ROTATION;

  public static final double CENTIMETERS_PER_INCH = 2.54;
  public static final double INCHES_PER_FOOT = 12.0;
  public static final double CENTIMETERS_PER_METER = 100;
  public static final double METERS_PER_INCH = CENTIMETERS_PER_INCH / CENTIMETERS_PER_METER;
  public static final double METERS_PER_FOOT = METERS_PER_INCH * INCHES_PER_FOOT;

  public static final double MILLISECONDS = 0.001;
}
