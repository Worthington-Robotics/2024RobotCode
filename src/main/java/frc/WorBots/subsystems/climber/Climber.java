// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.climber;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.subsystems.climber.ClimberIO.ClimberIOInputs;
import java.util.Optional;
import java.util.function.Supplier;

/** The climber subsystem of the robot */
public class Climber extends SubsystemBase {
  private ClimberIO io;
  private ClimberIOInputs inputs = new ClimberIOInputs();

  private Optional<Double> setpoint = Optional.empty();
  private double setpointDirection = 1.0;

  /** A supplier for the climber volts in manual mode */
  private Supplier<Double> manualClimberVolts = () -> 0.0;

  /** Whether to do climb locking */
  private boolean isClimbLocked = false;

  private static final double CLIMBING_FEEDFORWARD = -0.24;
  private static final double CONTROL_GAIN = 7.5;
  public static final double POSE_DEPLOY = 700.0;
  public static final double POSE_DROP = 600.0;
  public static final double POSE_FULL_CLIMB = -300.0;

  // Publishers

  /**
   * Constructs an instance of the climber.
   *
   * @param io The IO interface to be used.
   */
  public Climber(ClimberIO io) {
    this.io = io;
    if (RobotBase.isReal()) { // Real
    } else { // Sim
    }
  }

  /** The function that runs once per cycle. */
  public void periodic() {
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      setClimberVoltageRaw(0.0);
      manualClimberVolts = () -> 0.0;
    } else {
      if (setpoint.isPresent()) {
        final double setpoint2 = setpoint.get();
        double output = 0.0;
        if (setpointDirection == 1.0 && inputs.climber.positionRads < setpoint2) {
          output = CONTROL_GAIN;
        } else if (setpointDirection == -1.0 && inputs.climber.positionRads > setpoint2) {
          output = -CONTROL_GAIN;
        }
        setClimberVoltageRaw(output);
      } else {
        double climberVolts = manualClimberVolts.get();
        if (isClimbLocked) {
          climberVolts += CLIMBING_FEEDFORWARD;
        }
        setClimberVoltageRaw(climberVolts);
      }
    }

    // Publish motor inputs
    inputs.climber.publish();
  }

  /**
   * Sets and logs the climber voltage while bypassing software limits
   *
   * @param volts The climber voltage
   */
  private void setClimberVoltageRaw(double volts) {
    io.setClimberVoltage(volts);
  }

  /**
   * Sets the desired voltage for manual climbing
   *
   * @param volts The desired voltage
   */
  public void setManualClimberVolts(double volts) {
    manualClimberVolts = () -> volts;
  }

  /**
   * Sets the desired voltage for manual climbing
   *
   * @param volts The desired voltage
   */
  public void setManualClimberVolts(Supplier<Double> supplier) {
    manualClimberVolts = supplier;
  }

  public void setClimbLocked(boolean locked) {
    this.isClimbLocked = locked;
  }

  public void setSetpoint(double setpoint, double direction) {
    this.setpoint = Optional.of(setpoint);
    this.setpointDirection = direction;
  }

  public void clearSetpoint() {
    setpoint = Optional.empty();
    this.setpointDirection = 0.0;
  }

  public Command runPose(double setpoint, double direction) {
    return Commands.startEnd(() -> setSetpoint(setpoint, direction), () -> clearSetpoint());
  }

  public boolean isNearLimit() {
    return Math.abs(POSE_FULL_CLIMB - inputs.climber.positionRads) < 70.0;
  }
}
