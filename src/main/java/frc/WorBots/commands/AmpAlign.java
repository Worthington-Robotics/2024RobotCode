// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeneralMath;
import java.util.function.Supplier;

/** Command for teleop that drives the robot to the amp */
public class AmpAlign extends Command {
  private Drive drive;
  private final DriveController driveController = new DriveController();
  private final ProfiledPIDController drivePID =
      new ProfiledPIDController(
          1.9,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Units.inchesToMeters(50.0), Units.inchesToMeters(90.0)));
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(
          4.75,
          0.0,
          0.05,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(50.0), Units.degreesToRadians(720.0)));
  private Supplier<Double> ySupplier;

  public AmpAlign(Drive drive, Supplier<Double> ySupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.ySupplier = ySupplier;
    drivePID.setTolerance(Units.inchesToMeters(1.5));
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
    turnPID.setTolerance(Units.degreesToRadians(2));
  }

  @Override
  public void initialize() {
    drivePID.setGoal(
        AllianceFlipUtil.apply(
            (FieldConstants.Amp.openingStartX + FieldConstants.Amp.openingEndX) / 2.0));
    turnPID.setGoal(0);
  }

  @Override
  public void execute() {
    final double x = GeneralMath.clampMagnitude(drivePID.calculate(drive.getPose().getX()), 1.0);
    final double y = ySupplier.get();
    final double theta =
        GeneralMath.clampMagnitude(turnPID.calculate(drive.getYaw().getRadians()), 1.0);

    final var speeds =
        driveController.getSpeeds(
            x, y, theta, drive.getPose().getRotation(), drive.getMaxLinearSpeedMetersPerSec());
    // speeds.vxMetersPerSecond = x;
    // speeds.omegaRadiansPerSecond = theta;
    driveController.drive(drive, speeds);
  }
}
