// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.lights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.math.GeneralMath;
import frc.WorBots.util.math.InterpolatingTable;

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

  /** Creates a flame with the specified height */
  public static void flame(LightsIO io, double height, ColorSequence colors) {
    final double flicker = 0.25;
    final double scale = height * (1.0 - flicker) + Math.random() * flicker;
    for (int i = 0; i < io.getCount(); i++) {
      final double scalarPosition = (double) i / io.getCount();
      final double pos = scalarPosition / scale;
      if (pos > 1.0) {
        io.setLED(i, Color.kBlack);
      } else {
        final Color color = colors.sample(pos);
        io.setLED(i, color);
      }
    }
  }

  /** State for a two-color morphing lava pattern */
  public static class Lava {
    /** The speed at which the bubbles move */
    private static final double WALK_SPEED = 0.00006;

    /** The speed at which the bubbles fade in and out */
    private static final double BUBBLE_SPEED = 0.00008;

    /** The maximum velocity for walk and bubble. Needed so that velocities don't go wildly fast */
    private static final double MAX_VELOCITY = 0.015;

    /** The size of each point. Larger values make the points smaller */
    private static final double DISTANCE_SCALING = 21.0;

    /**
     * The bias between color 1 and color 2. Will have to be turned down when more points are added
     */
    private static final double BIAS = 0.3;

    /** The number of points in the lava */
    private static final int POINT_COUNT = 12;

    /** A single point/bubble in the lava */
    private class Point {
      double pos;
      double magnitude;
      double posVel;
      double magVel;

      public Point(double pos, double magnitude) {
        this.pos = pos;
        this.magnitude = magnitude;
        // Start with a bit of motion
        this.posVel = MAX_VELOCITY * (Math.random() - 0.5) / 4.0;
        this.magVel = MAX_VELOCITY * (Math.random() - 0.5) / 4.0;
      }
    }

    private Point[] points = new Point[POINT_COUNT];

    public Lava() {
      // Start all points at evenly spaced positions and 0.5 magnitude
      for (int i = 0; i < POINT_COUNT; i++) {
        points[i] = new Point((double) i / POINT_COUNT, 0.5);
      }
    }

    public void run(LightsIO io, ColorSequence colors) {
      // Move the position and magnitude of the points by random walking their
      // velocities and
      // integrating down to position
      for (Point point : points) {
        point.posVel += WALK_SPEED * (Math.random() - 0.5);
        point.posVel = GeneralMath.clampMagnitude(point.posVel, MAX_VELOCITY);
        point.pos += point.posVel;
        point.pos = MathUtil.inputModulus(point.pos, 0, 1);

        point.magVel += BUBBLE_SPEED * (Math.random() - 0.5);
        point.magVel = GeneralMath.clampMagnitude(point.magVel, MAX_VELOCITY);
        point.magnitude += point.magVel;
        point.magnitude = MathUtil.clamp(point.magnitude, 0, 1);
        // Prevent the velocity from straying out of range for a long time by kicking it
        // in the opposite direction when at the magnitude border
        if (point.magnitude == 0) {
          point.magVel = BUBBLE_SPEED / 2.0;
        }
        if (point.magnitude == 1.0) {
          point.magVel = -(BUBBLE_SPEED / 2.0);
        }
      }

      // Set the lights
      for (int i = 0; i < io.getCount(); i++) {
        final double scalarPosition = (double) i / io.getCount();

        // Sum all the points into this light's color
        double sum = 0.0;
        for (Point point : points) {
          final double distance = GeneralMath.wrappingDifference(scalarPosition, point.pos, 1.0);
          final double value =
              MathUtil.clamp(
                  point.magnitude - DISTANCE_SCALING * Math.pow(distance, 2.0), 0.0, 1.0);
          sum += value * BIAS;
        }
        final Color color = colors.sample(sum);
        io.setLED(i, color);
      }
    }
  }

  /**
   * Does a linear interpolation between two colors
   *
   * @param c1 The first color
   * @param c2 The second color
   * @param t The scalar interpolation factor, from 0 to 1
   * @return
   */
  public static Color colorLerp(Color c1, Color c2, double t) {
    t = MathUtil.clamp(t, 0, 1);

    final double r = GeneralMath.scale(t, c1.red, c2.red);
    final double g = GeneralMath.scale(t, c1.green, c2.green);
    final double b = GeneralMath.scale(t, c1.blue, c2.blue);

    return new Color(r, g, b);
  }

  /** A sequence of two or more colors that can be linearly interpolated between */
  public static class ColorSequence {
    private final InterpolatingTable red;
    private final InterpolatingTable green;
    private final InterpolatingTable blue;

    public ColorSequence(Color... colors) {
      final int colorCount = colors.length;

      double[][] redTable = new double[colorCount][2];
      double[][] greenTable = new double[colorCount][2];
      double[][] blueTable = new double[colorCount][2];

      for (int i = 0; i < colorCount; i++) {
        final double position = (double) i / colorCount;
        redTable[i] = new double[] {position, colors[i].red};
        greenTable[i] = new double[] {position, colors[i].green};
        blueTable[i] = new double[] {position, colors[i].blue};
      }

      red = new InterpolatingTable(redTable);
      green = new InterpolatingTable(greenTable);
      blue = new InterpolatingTable(blueTable);
    }

    public Color sample(double t) {
      t = MathUtil.clamp(t, 0.0, 1.0);
      return new Color(red.get(t), green.get(t), blue.get(t));
    }
  }
}
