package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;

public class Superstructure extends SubsystemBase {
  private SuperstructureIO io;
  private SuperstructureIOInputs inputs = new SuperstructureIOInputs();

  private ProfiledPIDController pivotController;
  private ProfiledPIDController elevatorController;
  private ElevatorFeedforward elevatorFeedForward;
  private ArmFeedforward pivotFeedForward;

  public Superstructure(SuperstructureIO io) {
    this.io = io;

    if (!Constants.getSim()) {

    } else {

    }
  }

  public void periodic() {
    io.updateInputs(inputs);
  }
}
