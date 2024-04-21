// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.AllianceFlipUtil;
import java.util.function.Supplier;

/**
 * A command that controls the drive rotation, pivot, and shooter RPM to do passes from the source
 * to the wing
 */
public class WingPass extends Command {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Shooter shooter;
  private final DriveController driveController = new DriveController();
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;

  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(
          4.05,
          0.0,
          0.05,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(130.0), Units.degreesToRadians(720.0)),
          Constants.ROBOT_PERIOD);

  public WingPass(
      Drive drive,
      Superstructure superstructure,
      Shooter shooter,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.shooter = shooter;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    addRequirements(drive, superstructure, shooter);
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    double goal = Units.degreesToRadians(147 + 180);
    if (AllianceFlipUtil.shouldFlip()) {
      goal *= -1;
    }
    turnPID.setGoal(goal);

    superstructure.setPose(Preset.WING_PASS);
    shooter.setSpeedVoid(3200);
  }

  @Override
  public void execute() {
    final double rv = turnPID.calculate(drive.getYaw().getRadians());
    final ChassisSpeeds driveSpeeds =
        driveController.getSpeeds(
            leftXSupplier.get(),
            leftYSupplier.get(),
            0.0,
            drive.getYaw(),
            drive.getMaxLinearSpeedMetersPerSec());
    drive.runVelocity(
        new ChassisSpeeds(driveSpeeds.vxMetersPerSecond, driveSpeeds.vyMetersPerSecond, rv));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    shooter.idle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
