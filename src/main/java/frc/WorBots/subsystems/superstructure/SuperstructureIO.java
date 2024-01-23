// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

public interface SuperstructureIO {
  public static class SuperstructureIOInputs {
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorTemp = 0.0;
    public double elevatorVoltage = 0.0;

    public double pivotPositionAbsRad = 0.0;
    public double pivotPositionRelRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotVoltageApplied = 0.0;
    public double pivotTemp = 0.0;
  }

  public default void setElevatorVoltage(double volts) {}

  public default void setPivotVoltage(double volts) {}

  public default void updateInputs(SuperstructureIOInputs inputs) {}
}
