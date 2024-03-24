// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.lights;

import edu.wpi.first.wpilibj.util.Color;
import frc.WorBots.util.cache.Cache.TimeCache;

/** Utility functions and mode classes for the control of the lights */
public class LightsUtil {
  /**
   * Sets all the lights to a single color
   *
   * @param io The lights to set
   * @param color The color
   */
  public static void solid(LightsIO io, Color color) {
    solid(io, color, 1.0);
  }

  /**
   * Sets a portion of the lights to a single color
   *
   * @param io The lights to set
   * @param color The color
   * @param percent The portion (0-1) to set
   */
  public static void solid(LightsIO io, Color color, double percent) {
    final int count = (int) (io.getCount() * percent);
    for (int i = 0; i < count; i++) {
      io.setLED(i, color);
    }
  }

  /** Make a rainbow pattern */
  public static void rainbow(LightsIO io, double cycleLength, double duration) {
    double x = (1 - ((TimeCache.getInstance().get() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < io.getCount(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= 0) {
        io.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  /** Make a two-color wave pattern */
  public static void wave(
      LightsIO io, Color c1, Color c2, double cycleLength, double duration, double waveExponent) {
    double x = (1 - ((TimeCache.getInstance().get() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < io.getCount(); i++) {
      x += xDiffPerLed;
      if (i >= 0) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        io.setLED(i, new Color(red, green, blue));
      }
    }
  }

  /** Make a bouncing pattern */
  public static void bounce(
      LightsIO io, Color c1, double cycleLength, double duration, double waveExponent) {
    double x = (Math.sin((TimeCache.getInstance().get() % (2 * Math.PI))) + 1) / 2;
    double xDiffPerLed = cycleLength;
    for (int i = 0; i < io.getCount(); i++) {
      x -= xDiffPerLed;
      if (i >= 0) {
        double red = (c1.red * (x));
        io.setLED(i, new Color(red, red, red));
      }
    }
  }
}
