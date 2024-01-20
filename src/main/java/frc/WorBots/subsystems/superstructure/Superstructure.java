package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;

public class Superstructure extends SubsystemBase {
  private SuperstructureIO io;
  private SuperstructureIOInputs inputs = new SuperstructureIOInputs();
  private SuperstructureState state = SuperstructureState.POSE;
  private SuperstructurePose.Preset setpoint = SuperstructurePose.Preset.AMP;
  private double pivotAbsAngleRad = 0.0;
  private double shootingAngleRad = 0.6;

  private ProfiledPIDController pivotController;
  private ProfiledPIDController elevatorController;
  private ElevatorFeedforward elevatorFeedForward;
  private ArmFeedforward pivotFeedForward;

  private enum SuperstructureState {
    POSE, SHOOTING
  }

  public Superstructure(SuperstructureIO io) {
    this.io = io;
    io.updateInputs(inputs);
    pivotAbsAngleRad = inputs.pivotPositionAbsRad;
    if (!Constants.getSim()) {
      pivotController = new ProfiledPIDController(0, 0, 0, null);
      elevatorController = new ProfiledPIDController(0, 0, 0, null);
    } else {
      pivotController = new ProfiledPIDController(0, 0, 0, null);
      elevatorController = new ProfiledPIDController(0, 0, 0, null);
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    switch (state) {
      case POSE:
        io.setElevatorVoltage(
            elevatorController.calculate(inputs.elevatorPositionMeters, setpoint.getVecPose().get(0, 0))
                + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec));
        io.setPivotVoltage(pivotController.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad,
            setpoint.getVecPose().get(1, 0))
            + pivotFeedForward.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec));
        break;
      case SHOOTING:
        io.setElevatorVoltage(elevatorController.calculate(inputs.elevatorPositionMeters, 0.0)
            + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec));
        io.setPivotVoltage(pivotController.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad, shootingAngleRad)
            + pivotFeedForward.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec));
        break;
    }
    if (DriverStation.isDisabled()) {
      io.setElevatorVoltage(0.0);
      io.setPivotVoltage(0.0);
    }
  }

  public void setShootingAngleRad(double angle) {
    shootingAngleRad = angle;
  }
}
