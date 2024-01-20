package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;

public class Superstructure extends SubsystemBase {
  private SuperstructureIO io;
  private SuperstructureIOInputs inputs = new SuperstructureIOInputs();
  private SuperstructureState state = SuperstructureState.POSE;
  private Vector<N2> setpoint = VecBuilder.fill(0.0, 0.0);

  private ProfiledPIDController pivotController;
  private ProfiledPIDController elevatorController;
  private ElevatorFeedforward elevatorFeedForward;
  private ArmFeedforward pivotFeedForward;

  private enum SuperstructureState {
    POSE, SHOOTING
  }

  public Superstructure(SuperstructureIO io) {
    this.io = io;

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

      break;
      case SHOOTING:
      break;
    }
    if (DriverStation.isDisabled()) {
      io.setElevatorVoltage(0.0);
      io.setPivotVoltage(0.0);
    }
  }
}
