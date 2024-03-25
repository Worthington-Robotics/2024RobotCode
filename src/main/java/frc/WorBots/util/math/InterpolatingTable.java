// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

/**
 * A 1-1 2D lookup table for doubles that linearly interpolates between values not in the table
 *
 * <p>Mostly from
 * https://github.com/FRC-5013-Park-Hill-Robotics/5013-RapidReact/blob/main/src/main/java/frc/robot/trobot5013lib/LinearInterpolator.java
 */
public class InterpolatingTable {
  private final double[][] table;

  /**
   * Constructs a new interpolating table
   *
   * @param table Must be an array of arrays with two doubles. The first double is the input, and
   *     the second double is the output for that sample. The list of samples must also be arranged
   *     with the inputs in ascending order
   */
  public InterpolatingTable(double[][] table) {
    this.table = table;
  }

  /**
   * Gets an interpolated value from the table.
   *
   * @param x The input value
   * @return The output value from the table, interpolated if the input does not exist as a sample
   */
  public double get(double x) {
    int index = 0;
    for (index = 0; index < table.length; index++) {
      if (table[index][0] >= x) {
        break;
      }
    }

    if (index >= table.length) {
      return table[table.length - 1][1];
    }

    final double high_y = table[index][1];
    final double high_x = table[index][0];
    if ((high_x == x) || (index == 0)) {
      return high_y;
    }
    final double low_y = table[index - 1][1];
    final double low_x = table[index - 1][0];

    return (low_y + (x - low_x) * (high_y - low_y) / (high_x - low_x));
  }
}
