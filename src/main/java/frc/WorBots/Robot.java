// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.Cache.AllianceCache;
import frc.WorBots.util.Cache.TimeCache;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.AllianceFlipUtil;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.WorBots.util.debug.Logger;
// import frc.WorBots.util.math.GeomUtil;
// import frc.WorBots.util.math.ShooterMath;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private PowerDistribution pdp;

  // private UsbCamera camera;

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Lights.getInstance();
    pdp = new PowerDistribution();

    robotContainer = new RobotContainer();
    StatusPage.reportStatus(StatusPage.ROBOT_CODE, true);

    if (Constants.getSim()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(false);
    }

    this.addPeriodic(
        () -> {
          // Simple status updates
          StatusPage.periodic(pdp);
          // StatusPage.reportStatus(
          //     StatusPage.DRIVER_CAM,
          //     camera.isConnected() && camera.isEnabled() && camera.isValid());
          StatusPage.reportStatus(
              StatusPage.DRIVE_CONTROLLER, robotContainer.driver.getHID().isConnected());
          StatusPage.reportStatus(
              StatusPage.OPERATOR_CONTROLLER, robotContainer.operator.getHID().isConnected());
        },
        kDefaultPeriod);

    // camera = CameraServer.startAutomaticCapture();
    TimeCache.getInstance().update();
    AllianceCache.getInstance().update();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // ==== DISABLE BEFORE COMP ====
    // final var robotPose = robotContainer.drive.getPose();
    // final var robotSpeeds = robotContainer.drive.getFieldRelativeSpeeds();
    // final double range = ShooterMath.getGoalDistance(robotPose);
    // SmartDashboard.putNumber("Speaker Range", range);
    // final var goalPose = ShooterMath.getGoal();
    // SmartDashboard.putNumberArray("Goal Pose", Logger.translation2dToArray(goalPose));
    // var goalAngle = ShooterMath.getGoalToRobotAngle(robotPose);
    // SmartDashboard.putNumber("Goal to Robot Angle", goalAngle.getRadians());

    // var shotData = ShooterMath.calculateShotData(robotPose, robotSpeeds);
    // SmartDashboard.putNumber("Robot.java Shot Angle", shotData.pivotAngle());
    // SmartDashboard.putString("Shot Confidence", shotData.confidence().toString());
    // SmartDashboard.putNumber("Calculated Shooter RPM", shotData.rpm());
    // final var adjusted = new Pose2d(robotPose.getTranslation(), shotData.robotAngle());
    // SmartDashboard.putNumberArray("Adjusted Drive Pose", Logger.pose2dToArray(adjusted));

    // var nextRobotPose = GeomUtil.applyChassisSpeeds(robotPose, robotSpeeds,
    // Constants.ROBOT_PERIOD);
    // SmartDashboard.putNumberArray("Next Robot Pose", Logger.pose2dToArray(nextRobotPose));
    // ==========

    // Update caches
    TimeCache.getInstance().update();
    AllianceCache.getInstance().update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (Constants.IS_COMP) {
      Lights.getInstance().setMode(LightsMode.Status);
    } else {
      // Display red to show whether the shooter has a game piece, for loading
      if (robotContainer.shooter.hasGamePiece()) {
        Lights.getInstance().setMode(LightsMode.Delivery);
      } else {
        Lights.getInstance().setMode(LightsMode.RedBlue);
      }
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    AllianceFlipUtil.apply(0.0);
    robotContainer.drive.stop();
    robotContainer.intake.setVolts(0.0);
    robotContainer.shooter.setRawFlywheelSpeed(0);
    robotContainer.shooter.setRawFeederVolts(0.0);
    robotContainer.superstructure.setModeVoid(SuperstructureState.DISABLED);
    if (Constants.IS_COMP) {
      Lights.getInstance().setMode(LightsMode.Alliance);
    } else {
      Lights.getInstance().setMode(LightsMode.MatchTime);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.drive.stop();
    robotContainer.intake.setVolts(0.0);
    robotContainer.shooter.setRawFlywheelSpeed(0);
    robotContainer.shooter.setRawFeederVolts(0.0);
    robotContainer.superstructure.setModeVoid(SuperstructureState.DISABLED);

    Lights.getInstance().setMode(LightsMode.Delivery);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
