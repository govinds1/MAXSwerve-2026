package frc.robot;

import edu.wpi.first.math.util.Units;

public class Helpers {
  public static double modRotations(double input) {
    input %= 1.0;
    if (input < 0.0) {
      input += 1.0;
    }
    return input;
  }

  public static double modDegrees(double input) {
    input %= 360.0;
    if (input < 0.0) {
      input += 360.0;
    }
    return input;
  }

  public static double modRadians(double input) {
    return Units.degreesToRadians(modDegrees(Units.radiansToDegrees(input)));
  }

  public static int clamp(int val, int min, int max) {
    return Math.max(min, Math.min(max, val));
  }

  public static double signedSquare(double input) {
    // Multiply input by absolute value of itself.
    return input * Math.abs(input);
  }
}