// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.MathUtil;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ShooterIOTalon implements ShooterIO {
  private TalonFX topFlywheel;
  private TalonFX bottomFlywheel;
  private TalonFX feederWheel;
  private TimeOfFlight timeOfFlight;

  private final TalonSignalsPositional feederWheelSignals;

  public ShooterIOTalon() {
    topFlywheel = new TalonFX(15);
    bottomFlywheel = new TalonFX(16);
    feederWheel = new TalonFX(17);
    timeOfFlight = new TimeOfFlight(18);
    timeOfFlight.setRangingMode(RangingMode.Short, 24);

    feederWheelSignals = new TalonSignalsPositional(feederWheel);
    feederWheel.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    feederWheelSignals.update(inputs.feederWheel, feederWheel);

    inputs.velocityRPMBottom = bottomFlywheel.getVelocity().getValue() * 60;
    inputs.velocityRPMTop = topFlywheel.getVelocity().getValue() * 60;

    inputs.timeOfFlightDistanceMeters = timeOfFlight.getRange() / 1000.0;
  }

  @Override
  public void setBottomFlywheelVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    bottomFlywheel.setVoltage(volts);
  }

  @Override
  public void setTopFlywheelVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    topFlywheel.setVoltage(volts);
  }

  @Override
  public void setFeederWheelVoltage(double volts) {
    volts = MathUtil.clamp(volts, -10, 10);
    feederWheel.setVoltage(volts);
  }
}
