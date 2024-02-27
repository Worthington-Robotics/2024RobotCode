// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** An explicitly updated cache for a value */
public class Cache<T> {
  private final Supplier<T> supplier;
  private T cachedValue;

  public Cache(Supplier<T> supplier) {
    this.supplier = supplier;
  }

  /**
   * Gets the cached value. Will only update if the cached value is null
   *
   * @return The cached value
   */
  public T get() {
    if (cachedValue == null) {
      update();
    }
    return cachedValue;
  }

  /** Update the cached value */
  public void update() {
    cachedValue = supplier.get();
  }

  public static class TimeCache extends Cache<Double> {
    private static TimeCache instance = new TimeCache();

    public static TimeCache getInstance() {
      return instance;
    }

    private TimeCache() {
      super(() -> Timer.getFPGATimestamp());
    }
  }
}
