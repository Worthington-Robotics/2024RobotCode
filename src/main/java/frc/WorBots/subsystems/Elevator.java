// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private TalonFX leftMotor = new TalonFX(3);
  private TalonFX rightMotor = new TalonFX(2);

  private double demand = 0.0;

  public Elevator() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);

    rightMotor.setInverted(false);
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      rightMotor.setVoltage(0.0);
    } else {
      rightMotor.setVoltage(demand);
    }
    SmartDashboard.putNumber("Elevator/Demand", demand);
    SmartDashboard.putNumber("Elevator/Encoder", getEncoder());
    SmartDashboard.putBoolean("Elevator/Bottom Limit", getBottomLimitTriggered());
  }

  public void setDemand(double demand) {
    this.demand = demand;
  }

  public boolean getBottomLimitTriggered() {
    return rightMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public double getEncoder() {
    return rightMotor.getRotorPosition().getValueAsDouble();
  }

  public Command setDemandCommand(double demand) {
    return this.runEnd(() -> this.demand = demand, () -> this.demand = 0.0);
  }
}
