// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class IntakeIOTalon implements IntakeIO {
  private final TalonFX intakeMotor;
  private TimeOfFlight timeOfFlight;

  private final TalonSignalsPositional motorSignals;

  public IntakeIOTalon() {
    intakeMotor = new TalonFX(1);
    // timeOfFlight = new TimeOfFlight(0);
    // timeOfFlight.setRangingMode(RangingMode.Short, 24);

    motorSignals = new TalonSignalsPositional(intakeMotor);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorSignals.update(inputs.motor, intakeMotor);
    inputs.isConnected = inputs.motor.isConnected;

    // inputs.timeOfFlightDistanceMeters = timeOfFlight.getRange() / 1000;
    // inputs.isConnected &= DeviceUtils.getTimeOfFlightStatus(timeOfFlight);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    volts = MathUtil.clamp(volts, -10, 10);
    intakeMotor.setVoltage(volts);
  }
}
