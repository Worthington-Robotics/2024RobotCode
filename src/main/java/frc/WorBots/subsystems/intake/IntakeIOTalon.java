// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.filter.LinearFilter;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class IntakeIOTalon implements IntakeIO {
  private final TalonFX intakeMotor;
  private TimeOfFlight timeOfFlight;

  private final TalonSignalsPositional motorSignals;
  private final StatusSignal<Double> currentDrawSignal;

  private final LinearFilter tofFilter = LinearFilter.movingAverage(1);

  public IntakeIOTalon() {
    intakeMotor = new TalonFX(1);
    timeOfFlight = new TimeOfFlight(13);
    timeOfFlight.setRangingMode(RangingMode.Short, 24);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setInverted(false);

    motorSignals = new TalonSignalsPositional(intakeMotor);
    currentDrawSignal = intakeMotor.getTorqueCurrent();
    currentDrawSignal.setUpdateFrequency(100.0);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorSignals.update(inputs.motor, intakeMotor);
    StatusSignal.refreshAll(currentDrawSignal);
    inputs.isConnected = inputs.motor.isConnected;

    inputs.timeOfFlightDistanceMeters = tofFilter.calculate(timeOfFlight.getRange()) / 1000;
    inputs.currentDraw = currentDrawSignal.getValue();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motorSignals.setVoltage(intakeMotor, volts, 10);
  }
}
