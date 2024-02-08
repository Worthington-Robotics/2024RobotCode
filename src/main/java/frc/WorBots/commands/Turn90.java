// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.math.GeneralMath;
import frc.WorBots.util.math.GeneralMath.SnapDirection;

/** Command that rotates the drive to 90 degree absolute snaps */
public class Turn90 extends Command {
  private final Drive drive;
  private final boolean isRight;
  private final SnapDirection direction;

  /**
   * The amount that the theta setpoint will be extended in its direction. This allows the turns to
   * consistently go in the right direction and prevents snapping to the wrong angle when you are
   * not quite straight
   */
  private static final Rotation2d threshold = Rotation2d.fromDegrees(4);

  /**
   * Constructs a 90 turn command
   *
   * @param drive The drive
   * @param isRight Whether to turn right or left
   */
  public Turn90(Drive drive, boolean isRight) {
    this.drive = drive;
    this.isRight = isRight;
    direction = isRight ? SnapDirection.Negative : SnapDirection.Positive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Calculate the 90 turn angle
    Rotation2d angle = drive.getRotation();
    if (isRight) {
      angle = angle.minus(threshold);
    } else {
      angle = angle.plus(threshold);
    }
    angle = GeneralMath.snapRotation(angle, Rotation2d.fromDegrees(90), direction);

    // Apply to the drivebase
    drive.setSingleThetaSetpoint(angle);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
