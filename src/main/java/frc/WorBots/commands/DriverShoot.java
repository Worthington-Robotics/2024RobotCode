package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;

public class DriverShoot extends Command {
  private Drive drive;
  private Superstructure superstructure;
  private Supplier<Double> leftXSupplier;
  private Supplier<Double> leftYSupplier;
  private Supplier<Double> aimLeftXSupplier;
  private Supplier<Double> aimLeftYSupplier;

  private static final double minAngleRads = 0.0;
  private static final double maxAngleRads = 0.0;

  public DriverShoot(Drive drive, Superstructure superstructure, Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier, Supplier<Double> aimLeftXSupplier, Supplier<Double> aimLeftYSupplier) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.aimLeftXSupplier = aimLeftXSupplier;
    this.aimLeftYSupplier = aimLeftYSupplier;
  }

  @Override
  public void initialize() {
    superstructure.setMode(SuperstructureState.SHOOTING);
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightY = aimLeftYSupplier.get();
    double rightX = aimLeftXSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, 0.05);
    rightY = MathUtil.applyDeadband(rightY, 0.05);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Calcaulate new linear components
    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(new Translation2d(linearMagnitude, new Rotation2d()), new Rotation2d()))
        .getTranslation();

    // Convert to meters per second
    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), rightY * 5.0);

    // Convert from field relative
    var driveRotation = drive.getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
    }
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond, driveRotation);

    // Send to drive
    // var driveTranslation = FlipFieldUtil.apply(drive.getPose().getTranslation());
    if (Math.abs(speeds.vxMetersPerSecond) < 1e-3 && Math.abs(speeds.vyMetersPerSecond) < 1e-3
        && Math.abs(speeds.omegaRadiansPerSecond) < 1e-3) {
      drive.stop();
    } else {
      drive.runVelocity(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setMode(SuperstructureState.POSE);
  }
}
