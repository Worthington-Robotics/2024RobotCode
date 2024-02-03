// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class ShooterIOKraken implements ShooterIO {
  private TalonFX topFlywheel;
  private TalonFX bottomFlywheel;
  private TalonFX feederWheel;
  private TimeOfFlight timeOfFlight;

  public ShooterIOKraken() {
    topFlywheel = new TalonFX(0);
    bottomFlywheel = new TalonFX(0);
    feederWheel = new TalonFX(0);
    timeOfFlight = new TimeOfFlight(0);
    timeOfFlight.setRangingMode(RangingMode.Short, 24);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPMBottom = bottomFlywheel.getVelocity().getValue() * 60;
    inputs.velocityRPMTop = topFlywheel.getVelocity().getValue() * 60;

    inputs.feederWheelPositionRads = Units.rotationsToRadians(feederWheel.getPosition().getValue());
    inputs.feederWheelVelocityRadPerSec =
        Units.rotationsToRadians(feederWheel.getVelocity().getValue());
    inputs.feederWheelCurrentAmps = feederWheel.getStatorCurrent().getValue();
    inputs.timeOfFlightDistanceMeters = timeOfFlight.getRange() / 1000.0;
  }

  @Override
  public void setBottomFlywheelVolts(double volts) {
    MathUtil.clamp(volts, -12, 12);
    bottomFlywheel.setVoltage(volts);
  }

  @Override
  public void setTopFlywheelVolts(double volts) {
    MathUtil.clamp(volts, -12, 12);
    topFlywheel.setVoltage(volts);
  }

  @Override
  public void setFeederWheelVoltage(double volts) {
    MathUtil.clamp(volts, -10, 10);
    feederWheel.setVoltage(volts);
  }
}
