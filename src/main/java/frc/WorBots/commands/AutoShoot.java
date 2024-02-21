// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.GeneralMath;
import frc.WorBots.util.math.ShooterMath;
import java.util.function.*;

public class AutoShoot extends SequentialCommandGroup {
  // Locations
  private static double speakerOpeningHeightZ;
  private static double speakerOpeningCenterY;
  private Supplier<Double> leftXSupplier;
  private Supplier<Double> leftYSupplier;
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          3.75,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(140.0), Units.degreesToRadians(720.0)),
          0.02);

  /**
   * This command automatically drives to a known safe shooting location and shoots a game piece.
   *
   * @param superstructure The superstructure subsystem.
   * @param drive The drive subsystem.
   */
  public AutoShoot(
      Superstructure superstructure,
      Drive drive,
      Shooter shooter,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier) {
    addRequirements(superstructure, drive);
    speakerOpeningHeightZ =
        (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower)
            / 2;
    speakerOpeningCenterY = (FieldConstants.Speaker.speakerY);
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.5));

    Supplier<Double> pivotAngle =
        () -> {
          Pose2d robotPose = drive.getPose();
          var shotData = ShooterMath.calculateShotData(robotPose, drive.getFieldRelativeSpeeds());
          SmartDashboard.putNumber("Shot Angle", shotData.pivotAngle());
          return shotData.pivotAngle();
        };
    Supplier<Rotation2d> driveAngleSupplier =
        () -> {
          Pose2d robotPose = drive.getPose();
          double robotAngle =
              ShooterMath.getRobotAngle(robotPose, drive.getFieldRelativeSpeeds()).getRadians();
          return new Rotation2d(robotAngle);
        };
    Supplier<Double> shooterSpeedSupplier =
        () -> {
          Pose2d robotPose = drive.getPose();
          return ShooterMath.calculateShooterRPM(robotPose);
        };
    Supplier<ChassisSpeeds> speedsSupplier =
        () -> {
          SmartDashboard.putNumber("Controller Error", thetaController.getPositionError());
          Pose2d robotPose = drive.getPose();
          double x = leftXSupplier.get();
          double y = leftYSupplier.get();
          // Get direction and magnitude of linear axes
          double linearMagnitude = Math.hypot(x, y);
          Rotation2d linearDirection = new Rotation2d(x, y);

          // Apply deadband
          linearMagnitude = MathUtil.applyDeadband(linearMagnitude, DriveController.deadband);

          // Apply squaring
          linearMagnitude = GeneralMath.curve(linearMagnitude, DriveController.driveCurveAmount);

          // Calcaulate new linear components
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(
                      new Transform2d(
                          new Translation2d(linearMagnitude, new Rotation2d()), new Rotation2d()))
                  .getTranslation();

          double thetaVelocity =
              thetaController.calculate(
                  robotPose.getRotation().getRadians(), driveAngleSupplier.get().getRadians());
          double thetaErrorAbs =
              Math.abs(robotPose.getRotation().minus(driveAngleSupplier.get()).getRadians());
          if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

          // Convert to meters per second
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX()
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * DriveController.driveSpeedMultiplier,
                  linearVelocity.getY()
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * DriveController.driveSpeedMultiplier,
                  thetaVelocity);

          // Convert to field relative based on the alliance
          var driveRotation = robotPose.getRotation();
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.vxMetersPerSecond,
                  speeds.vyMetersPerSecond,
                  speeds.omegaRadiansPerSecond,
                  driveRotation);
          return speeds;
        };
    var shooting =
        Commands.run(
            () -> {
              superstructure.setShootingAngleRad(pivotAngle);
            },
            superstructure);
    addCommands(
        superstructure.setMode(SuperstructureState.SHOOTING),
        shooting
            .alongWith(
                Commands.run(() -> shooter.spinToSpeedVoid(shooterSpeedSupplier.get()), shooter))
            .alongWith(Commands.run(() -> drive.runVelocity(speedsSupplier.get()), drive)),
        Commands.waitUntil(() -> false)
            .finallyDo(
                () -> {
                  superstructure.setModeVoid(SuperstructureState.DISABLED);
                  shooter.spinToSpeedVoid(0);
                }));
  }

  /**
   * Calculates the needed robot rotation in order to shoot into the speaker for a given pose,
   * automatically factors in alliance.
   *
   * @param robotPose The desired pose to be shot from, ignores rotation.
   * @return The rotation of the robot needed to shoot at the provided pose.
   */
  public static Rotation2d getRobotRotationToShoot(Pose2d robotPose) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    double translatedX = (isRed ? robotPose.getX() - FieldConstants.fieldLength : robotPose.getX());
    double robotY = robotPose.getY();
    double robotAngle = Math.atan2(robotY - (speakerOpeningCenterY), translatedX);
    return new Rotation2d(robotAngle);
  }
}
