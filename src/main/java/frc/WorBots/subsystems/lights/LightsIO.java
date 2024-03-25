// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Simple hardware wrapper around a set of addressable LEDs */
public class LightsIO {
  /** The number of LEDs */
  public static final int LIGHT_COUNT = 27;

  /** The ID of the LEDs */
  private static final int LIGHTS_ID = 9;

  /** Dimming factor to apply to the LED colors */
  private static final double DIMMING_FACTOR = 0.9;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  public LightsIO() {
    // Initialize lights and buffer
    leds = new AddressableLED(LIGHTS_ID);
    buffer = new AddressableLEDBuffer(LIGHT_COUNT);
    leds.setLength(LIGHT_COUNT);
    leds.start();
  }

  public void periodic() {
    // Dim all the LEDs
    for (int i = 0; i < LIGHT_COUNT; i++) {
      final Color color = buffer.getLED(i);
      final Color dimmed =
          new Color(
              color.red * DIMMING_FACTOR,
              color.green * DIMMING_FACTOR,
              color.blue * DIMMING_FACTOR);
      buffer.setLED(i, dimmed);
    }

    // Set the LEDs
    leds.setData(buffer);
  }

  /**
   * Wrapper function that sets an LED
   *
   * @param index The index of the LED
   * @param color The desired color
   */
  public void setLED(int index, Color color) {
    if (index >= LIGHT_COUNT) {
      return;
    }

    buffer.setLED(index, color);
  }

  /**
   * Wrapper function that sets an LED
   *
   * @param index The index of the LED
   * @param h The hue of the LED
   * @param s The saturation of the LED
   * @param v The value of LED
   */
  public void setHSV(int index, int h, int s, int v) {
    if (index >= LIGHT_COUNT) {
      return;
    }

    buffer.setHSV(index, h, s, v);
  }

  /** Gets the number of LEDs in the strip */
  public int getCount() {
    return LIGHT_COUNT;
  }
}
