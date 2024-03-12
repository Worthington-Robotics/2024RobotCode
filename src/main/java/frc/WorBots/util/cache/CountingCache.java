// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.cache;

import java.util.function.Supplier;

public class CountingCache<T> extends Cache<T> {
  private int count = 0;
  private final int maxCount;

  public CountingCache(Supplier<T> supplier, int maxCount) {
    super(supplier);
    this.maxCount = maxCount;
  }

  @Override
  public T get() {
    count++;
    if (count >= maxCount) {
      update();
      count = 0;
    }
    return super.get();
  }
}
