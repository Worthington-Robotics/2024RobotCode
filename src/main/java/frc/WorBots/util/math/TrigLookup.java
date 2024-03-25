// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.MathUtil;

/** Lookup tables for trig functions that don't need a lot of accuracy */
public class TrigLookup {
  private static final int RESOLUTION = 360;

  /** Lookup table of sine values */
  private static final double[] sinTable;

  static {
    sinTable = new double[RESOLUTION];
    for (int i = 0; i < RESOLUTION; i++) {
      final double radians = (double) i / (double) RESOLUTION * GeomUtil.PI2;
      sinTable[i] = Math.sin(radians);
    }
  }

  /**
   * Calculates the sin of an input
   *
   * @param radians The input in radians
   * @return The sin of the value
   */
  public static double sin(double radians) {
    radians = MathUtil.inputModulus(radians, 0, GeomUtil.PI2);
    final int index = (int) (radians / GeomUtil.PI2 * RESOLUTION);
    return sinTable[index];
  }

  /**
   * Calculates the cos of an input
   *
   * @param radians The input in radians
   * @return The cos of the value
   */
  public static double cos(double radians) {
    return sin(radians + Math.PI / 2.0);
  }
}
