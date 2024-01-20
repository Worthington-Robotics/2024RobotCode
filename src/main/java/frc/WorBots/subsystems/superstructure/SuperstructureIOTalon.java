package frc.WorBots.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class SuperstructureIOTalon implements SuperstructureIO {
  private final TalonFX elevator;
  private final boolean isElevatorInverted = false;

  private final TalonFX pivot;
  private final boolean isPivotInverted = false;
  private final DutyCycleEncoder pivotAbsEncoder;
  private final Encoder pivotRelEncoder;

  public SuperstructureIOTalon() {
    elevator = new TalonFX(0);
    pivot = new TalonFX(0);
    pivotAbsEncoder = new DutyCycleEncoder(0);
    pivotRelEncoder = new Encoder(0, 0);
    
    }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {

    
  }
}
