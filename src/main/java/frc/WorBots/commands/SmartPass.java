// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.InterpolatingTable;
import java.util.function.Supplier;

/**
 * A command that controls the drive rotation, pivot, and shooter RPM to do passes from anywhere on
 * the field to the wing
 */
public class SmartPass extends Command {
  /** Where the command is aiming to */
  private static final Translation2d GOAL = new Translation2d(0.0, FieldConstants.fieldWidth);

  /** Lookahead factor for aiming */
  private static final double LOOKAHEAD_FACTOR = 45.0;

  /** Lookup table for shot angle */
  private static final InterpolatingTable ANGLE_LOOKUP =
      new InterpolatingTable(
          new double[][] {
            {FieldConstants.midLineX, 0.720},
          });

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
          0.00,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(130.0), Units.degreesToRadians(720.0)),
          Constants.ROBOT_PERIOD);

  public SmartPass(
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

    superstructure.setMode(SuperstructureState.SHOOTING);
    shooter.setSpeedVoid(3200);
  }

  @Override
  public void execute() {
    // Do aim calculations
    final Pose2d robot =
        GeomUtil.applyChassisSpeeds(
            drive.getPose(), drive.getFieldRelativeSpeeds(), LOOKAHEAD_FACTOR);
    final Translation2d goal = AllianceFlipUtil.apply(GOAL);
    final double angle =
        Math.atan2(goal.getX() - robot.getX(), goal.getY() - robot.getY())
            + Units.degreesToRadians(90);

    final double distance = robot.getTranslation().getDistance(goal);
    final double pivotAngle = ANGLE_LOOKUP.get(distance);

    final double rv = turnPID.calculate(drive.getRotation().getRadians(), angle);
    final ChassisSpeeds driveSpeeds =
        driveController.getSpeeds(
            leftXSupplier.get(),
            leftYSupplier.get(),
            0.0,
            drive.getYaw(),
            drive.getMaxLinearSpeedMetersPerSec());
    drive.runVelocity(
        new ChassisSpeeds(driveSpeeds.vxMetersPerSecond, driveSpeeds.vyMetersPerSecond, rv));

    superstructure.setShootingAngleRad(pivotAngle);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    shooter.idle();
    superstructure.setPose(Preset.STOW);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
