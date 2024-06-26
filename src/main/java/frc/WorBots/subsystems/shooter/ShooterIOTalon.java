// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ShooterIOTalon implements ShooterIO {
  private final TalonFX topFlywheel;
  private final TalonFX bottomFlywheel;
  private final TalonFX feederWheel;
  private final TimeOfFlight timeOfFlight;

  private final TalonSignalsPositional topSignals;
  private final TalonSignalsPositional bottomSignals;
  private final TalonSignalsPositional feederWheelSignals;

  private final LinearFilter topFilter = LinearFilter.movingAverage(3);
  private final LinearFilter bottomFilter = LinearFilter.movingAverage(3);
  private final LinearFilter tofFilter = LinearFilter.movingAverage(1);

  public ShooterIOTalon() {
    topFlywheel = new TalonFX(7);
    bottomFlywheel = new TalonFX(6);
    feederWheel = new TalonFX(8);
    timeOfFlight = new TimeOfFlight(12);
    timeOfFlight.setRangingMode(RangingMode.Short, 24);

    topFlywheel.setNeutralMode(NeutralModeValue.Coast);
    bottomFlywheel.setNeutralMode(NeutralModeValue.Coast);
    feederWheel.setNeutralMode(NeutralModeValue.Brake);
    feederWheel.setInverted(false);
    bottomFlywheel.setInverted(false);

    topSignals = new TalonSignalsPositional(topFlywheel);
    bottomSignals = new TalonSignalsPositional(bottomFlywheel);
    feederWheelSignals = new TalonSignalsPositional(feederWheel);
    topFlywheel.optimizeBusUtilization();
    bottomFlywheel.optimizeBusUtilization();
    feederWheel.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topSignals.update(inputs.top, topFlywheel);
    bottomSignals.update(inputs.bottom, bottomFlywheel);
    feederWheelSignals.update(inputs.feederWheel, feederWheel);
    inputs.velocityRPMBottom =
        bottomFilter.calculate(
            Units.radiansPerSecondToRotationsPerMinute(inputs.bottom.velocityRadsPerSec));
    inputs.velocityRPMTop =
        topFilter.calculate(
            Units.radiansPerSecondToRotationsPerMinute(inputs.top.velocityRadsPerSec));

    inputs.timeOfFlightDistanceMeters = tofFilter.calculate(timeOfFlight.getRange()) / 1000.0;

    inputs.isConnected =
        inputs.top.isConnected && inputs.bottom.isConnected && inputs.feederWheel.isConnected;
  }

  @Override
  public void setBottomFlywheelVolts(double volts) {
    bottomSignals.setVoltage(bottomFlywheel, volts, 11);
  }

  @Override
  public void setTopFlywheelVolts(double volts) {
    topSignals.setVoltage(topFlywheel, volts, 11);
  }

  @Override
  public void setFeederWheelVoltage(double volts) {
    feederWheelSignals.setVoltage(feederWheel, volts, 10);
  }
}
