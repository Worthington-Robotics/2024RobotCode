// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ClimberIOTalon implements ClimberIO {
  private final TalonFX climber;

  private final TalonSignalsPositional climberSignals;

  public ClimberIOTalon() {
    climber = new TalonFX(20);

    climber.setNeutralMode(NeutralModeValue.Brake);
    climber.setPosition(0);
    climber.setInverted(true);

    climberSignals = new TalonSignalsPositional(climber);
    climber.optimizeBusUtilization();
  }

  public void setClimberVoltage(double volts) {
    climberSignals.setVoltage(climber, volts, 10);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    climberSignals.update(inputs.climber, climber);
  }
}
