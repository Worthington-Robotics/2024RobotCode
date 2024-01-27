// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

public interface ShooterIO {
  public static class ShooterIOInputs {
    boolean isConnected = false;
    public double velocityRPMTop = 0;
    public double velocityRPMBottom = 0;

    public double feederWheelPositionRads = 0.0;
    public double feederWheelVelocityRadPerSec = 0.0;
    public double feederWheelCurrentAmps = 0.0;
    public double timeOfFlightDistanceMeters = 0.0;
  }

  public default void setTopFlywheelVolts(double volts) {}

  public default void setBottomFlywheelVolts(double volts) {}

  public default void setFeederWheelVoltage(double volts) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}