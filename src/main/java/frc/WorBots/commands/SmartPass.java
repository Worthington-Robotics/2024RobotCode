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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.TunableDouble;
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
  private static final Translation2d GOAL = new Translation2d(1.0, FieldConstants.fieldWidth - 1.3);

  /** Lookahead factor for aiming */
  private static final double LOOKAHEAD_FACTOR = 1.34;

  /** Lookup table for shot angle */
  private static final InterpolatingTable ANGLE_LOOKUP =
      new InterpolatingTable(
          new double[][] {
            {FieldConstants.Wing.endX, 0.420},
            {FieldConstants.midLineX, 0.720},
            {AllianceFlipUtil.applyAgnostic(FieldConstants.Wing.endX), 0.720},
          });

  /** Lookup table for RPM */
  private static final InterpolatingTable RPM_LOOKUP =
      new InterpolatingTable(
          new double[][] {
            {FieldConstants.Wing.endX, 2300},
            {FieldConstants.midLineX, 3000},
            {AllianceFlipUtil.applyAgnostic(FieldConstants.Wing.endX), 3650},
          });

  /** RPM for straight shots */
  private static final TunableDouble STRAIGHT_RPM =
      new TunableDouble("Tuning", "Smart Pass", "Straight RPM", 3000);

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
    superstructure.setModeVoid(SuperstructureState.SHOOTING);
    turnPID.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    double rv = 0.0;

    // Refuse to pass when we are past the opponent's wing line
    if (AllianceFlipUtil.apply(drive.getPose().getX())
        >= AllianceFlipUtil.applyAgnostic(FieldConstants.Wing.endX)) {
      Lights.getInstance().setSolid(Color.kRed);
      shooter.idle();
      superstructure.setShootingAngleRad(Preset.HANDOFF.getPivot());
    } else {
      // Do aim calculations
      final Pose2d robot =
          GeomUtil.applyChassisSpeeds(
              drive.getPose(), drive.getFieldRelativeSpeeds(), LOOKAHEAD_FACTOR);
      final Translation2d goal = AllianceFlipUtil.apply(GOAL);
      // Get angle for the robot
      final double angle =
          Units.degreesToRadians(630.0)
              - Math.atan2(goal.getX() - robot.getX(), goal.getY() - robot.getY());

      // Choose what aiming to do based on whether we are blocked by the stage or not
      final boolean isBlocked = isBlockedByStage(drive.getPose());
      SmartDashboard.putBoolean("Smart Pass/Blocked", isBlocked);
      if (isBlocked) {
        // Lob over stage
        final double distance = robot.getTranslation().getDistance(goal);
        final double pivotAngle = ANGLE_LOOKUP.get(distance);
        superstructure.setShootingAngleRad(pivotAngle);
        final double rpm = RPM_LOOKUP.get(distance);
        shooter.setSpeedVoid(rpm);
        Lights.getInstance().setSolid(Color.kBlue);
      } else {
        // Shoot straight
        superstructure.setShootingAngleRad(Preset.STRAIGHT_PASS.getPivot());
        final double rpm = STRAIGHT_RPM.get();
        shooter.setSpeedVoid(rpm);
        Lights.getInstance().setSolid(Color.kPurple);
      }

      rv = turnPID.calculate(drive.getRotation().getRadians(), angle);
    }

    final ChassisSpeeds driveSpeeds =
        driveController.getSpeeds(
            leftXSupplier.get(),
            leftYSupplier.get(),
            0.0,
            drive.getYaw(),
            drive.getMaxLinearSpeedMetersPerSec());
    drive.runVelocity(
        new ChassisSpeeds(driveSpeeds.vxMetersPerSecond, driveSpeeds.vyMetersPerSecond, rv));
    SmartDashboard.putNumberArray("Smart Pass/Goal", Logger.translation2dToArray(GOAL));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    shooter.idle();
    superstructure.setPose(Preset.STOW);
    Lights.getInstance().setMode(LightsMode.Delivery);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /** Check if a pose is blocked by the stage for the pass */
  private boolean isBlockedByStage(Pose2d pose) {
    final double x = AllianceFlipUtil.apply(pose.getX());
    final double y = pose.getY();
    final boolean farBlocked =
        x > FieldConstants.Stage.foot2Center.getX() && y < FieldConstants.Stage.foot2Center.getY();
    final boolean sideBlocked =
        x > FieldConstants.Stage.foot1Center.getX() && y < FieldConstants.Stage.foot1Center.getY();
    return farBlocked || sideBlocked;
  }
}
