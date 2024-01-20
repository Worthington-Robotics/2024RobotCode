package frc.WorBots.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;
import frc.WorBots.util.Logger;

public class Superstructure extends SubsystemBase {
  private SuperstructureIO io;
  private SuperstructureIOInputs inputs = new SuperstructureIOInputs();
  private SuperstructureState state = SuperstructureState.POSE;
  private SuperstructurePose.Preset setpoint = SuperstructurePose.Preset.CLIMB;
  private double pivotAbsAngleRad = 0.0;
  private Supplier<Double> shootingAngleRad = () -> 0.5;

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
    if (!Constants.getSim()) { // Real
      pivotController = new ProfiledPIDController(8.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorController = new ProfiledPIDController(8.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorFeedForward = new ElevatorFeedforward(0.0, 0.0, 0.0);
      pivotFeedForward = new ArmFeedforward(0.0, 0.0, 0.0);
    } else { // Sim
      pivotController = new ProfiledPIDController(8.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorController = new ProfiledPIDController(8.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorFeedForward = new ElevatorFeedforward(0.0, 0.0, 0.0);
      pivotFeedForward = new ArmFeedforward(0.0, 0.0, 0.0);
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().setSuperstructureInputs(inputs);
    Logger.getInstance().setSuperstructureMode(state.name());
    switch (state) {
      case POSE:
        Logger.getInstance().setSuperstructureElevatorPosSetpoint(setpoint.getVecPose().get(0, 0));
        Logger.getInstance().setSuperstructurePivotPosSetpoint(setpoint.getVecPose().get(1, 0));
        double elevatorVoltage = elevatorController.calculate(inputs.elevatorPositionMeters,
            setpoint.getVecPose().get(0, 0))
            + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec);
        double pivotVoltage = pivotController.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad,
            setpoint.getVecPose().get(1, 0))
            + pivotFeedForward.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec);
        Logger.getInstance().setSuperstructurePivotVoltageSetpoint(pivotVoltage);
        Logger.getInstance().setSuperstructureElevatorVoltageSetpoint(elevatorVoltage);
        io.setElevatorVoltage(elevatorVoltage);
        io.setPivotVoltage(pivotVoltage);
        break;
      case SHOOTING:
        io.setElevatorVoltage(elevatorController.calculate(inputs.elevatorPositionMeters, 0.0)
            + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec));
        io.setPivotVoltage(pivotController.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad,
            shootingAngleRad.get())
            + pivotFeedForward.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec));
        break;
    }
    if (DriverStation.isDisabled()) {
      io.setElevatorVoltage(0.0);
      io.setPivotVoltage(0.0);
    }
  }

  public void setShootingAngleRad(double angle) {
    shootingAngleRad = () -> angle;
  }

  public void setShootingAngleRad(Supplier<Double> supplier) {
    shootingAngleRad = supplier;
  }

  public void setMode(SuperstructureState state) {
    this.state = state;
  }
}
