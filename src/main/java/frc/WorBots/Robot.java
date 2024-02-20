// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.ShooterMath;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private PowerDistribution pdp;

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
        },
        kDefaultPeriod);

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    final var robotPose = robotContainer.drive.getPose();
    final double range = ShooterMath.getGoalDistance(robotPose);
    SmartDashboard.putNumber("Speaker Range", range);
    final var goalPose = ShooterMath.getGoal();
    SmartDashboard.putNumberArray("Goal Pose", Logger.translation2dToArray(goalPose));
    final var theta = ShooterMath.getGoalTheta(robotPose);
    final var adjusted = new Pose2d(robotPose.getTranslation(), theta);
    SmartDashboard.putNumberArray("Adjusted Drive Pose", Logger.pose2dToArray(adjusted));
    var shotData = ShooterMath.calculateShotData(robotPose);
    SmartDashboard.putNumber("Robot.java Shot Angle", shotData.pivotAngle());
    var goalAngle = ShooterMath.getGoalToRobotAngle(robotPose);
    SmartDashboard.putNumber("Goal to Robot Angle", goalAngle.getRadians());
    var confidence = ShooterMath.getConfidence(robotPose);
    SmartDashboard.putString("Shot Confidence", confidence.toString());
    var rpm = ShooterMath.calculateShooterRPM(robotPose);
    SmartDashboard.putNumber("Calculated Shooter RPM", rpm);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
    robotContainer.superstructure.setMode(SuperstructureState.DISABLED);
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
    robotContainer.superstructure.setMode(SuperstructureState.DISABLED);
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
