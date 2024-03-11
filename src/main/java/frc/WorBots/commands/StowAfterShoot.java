// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;

/** Command that will stow the superstructure if some conditions are met */
public class StowAfterShoot extends InstantCommand {
  public StowAfterShoot(Superstructure superstructure) {
    super(
        () -> {
          if (shouldStow(superstructure)) {
            superstructure.setPose(Preset.STOW);
          }
        },
        superstructure);
  }

  /**
   * Checks whether the command should stow the superstructure
   *
   * @param superstructure The superstructure
   * @return Whether we should stow
   */
  private static boolean shouldStow(Superstructure superstructure) {
    if (superstructure.isInPose(Preset.SUBWOOFER_SHOOT) || superstructure.isShooting()) {
      return true;
    }

    return false;
  }
}
