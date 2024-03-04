// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.util.Units;
import frc.WorBots.util.debug.DebugValue;
import frc.WorBots.util.debug.DebugValue.DebugBool;
import frc.WorBots.util.debug.DebugValue.DebugDouble;
import frc.WorBots.util.math.GeneralMath;

/** Utility functions for working with hardware devices */
public class HardwareUtils {
  // Constants
  /** The maximum temperature in celsius that we want to run our motors at */
  public static final double maxMotorTemperature = 80.0;

  public static final double idealBatteryVoltage = 12.3;

  /**
   * Checks if a time of flight device is connected
   *
   * @param tof The time of flight
   * @return True if the device is working properly, false if it is not
   */
  public static boolean getTimeOfFlightStatus(TimeOfFlight tof) {
    final TimeOfFlight.Status status = tof.getStatus();
    return status == TimeOfFlight.Status.Valid;
  }

  /**
   * Safely sets a motor voltage with voltage limiting and temperature checks
   *
   * @param talon The motor
   * @param volts The voltage to set
   * @param maxVolts The maximum magnitude for the voltage
   * @param temp The temperature of the motor in celsius
   */
  public static void setTalonVoltage(TalonFX talon, double volts, double maxVolts, double temp) {
    if (temp < maxMotorTemperature) {
      volts = GeneralMath.clampMagnitude(volts, maxVolts);
      talon.setVoltage(volts);
    } else {
      talon.stopMotor();
    }
  }

  /**
   * Prepares a motor to be set as a leader
   *
   * @param talon The motor to prepare as a leader
   */
  public static void prepareLeader(TalonFX talon) {
    talon.getTorqueCurrent().setUpdateFrequency(50);
    talon.getDutyCycle().setUpdateFrequency(50);
  }

  /** Base inputs for a TalonFX */
  public static class TalonInputs {
    private final DebugDouble voltsPub;
    private final DebugDouble supplyVoltsPub;
    // private final DoublePublisher currentPub;
    // private final DoublePublisher tempPub;
    private final DebugBool connectedPub;

    public double appliedPowerVolts = 0.0;
    public double supplyVoltage = 0.0;
    public double currentDrawAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean isConnected = false;

    public TalonInputs(String table, String subtable) {
      voltsPub = DebugValue.compDouble(table, subtable + "/Applied Voltage");
      supplyVoltsPub = DebugValue.compDouble(table, subtable + "/Supply Voltage");
      connectedPub = DebugValue.compBool(table, subtable + "/Connected");
      // currentPub = this.table.getDoubleTopic("Current Draw").publish();
      // tempPub = this.table.getDoubleTopic("Temp Celsius").publish();
    }

    public void publish() {
      voltsPub.set(appliedPowerVolts);
      supplyVoltsPub.set(supplyVoltage);
      // currentPub.set(currentDrawAmps);
      // tempPub.set(temperatureCelsius);
      connectedPub.set(isConnected);
    }
  }

  /** Inputs for a TalonFX with position and velocity readings */
  public static class TalonInputsPositional extends TalonInputs {
    private final DebugDouble posPub;
    private final DebugDouble velPub;

    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;

    public TalonInputsPositional(String table, String subtable) {
      super(table, subtable);
      posPub = DebugValue.compDouble(table, subtable + "/Position");
      velPub = DebugValue.compDouble(table, subtable + "/Velocity");
    }

    public void publish() {
      super.publish();
      posPub.set(positionRads);
      velPub.set(velocityRadsPerSec);
    }
  }

  /** Base status signals for a TalonFX */
  public abstract static class TalonSignals {
    // private final StatusSignal<Double> tempSignal;
    private final StatusSignal<Double> voltsSignal;

    // private final StatusSignal<Double> currentSignal;

    public TalonSignals(TalonFX motor) {
      // tempSignal = motor.getDeviceTemp();
      voltsSignal = motor.getSupplyVoltage();
      // currentSignal = motor.getSupplyCurrent();

      // tempSignal.setUpdateFrequency(8);
      voltsSignal.setUpdateFrequency(20);
      // currentSignal.setUpdateFrequency(5);

      // For .get calls we need the duty cycle
      motor.getDutyCycle().setUpdateFrequency(50);
    }

    protected void refresh() {
      StatusSignal.refreshAll(voltsSignal);
    }

    public void update(TalonInputs inputs, TalonFX motor) {
      refresh();
      // inputs.temperatureCelsius = tempSignal.getValue();
      inputs.appliedPowerVolts = voltsSignal.getValue() * motor.get();
      inputs.supplyVoltage = voltsSignal.getValue();
      // inputs.currentDrawAmps = currentSignal.getValue();
      inputs.isConnected = motor.isAlive() && inputs.temperatureCelsius < maxMotorTemperature;
    }

    /**
     * Gets the motor supply voltage
     *
     * @return The supply voltage
     */
    public double getSupplyVoltage() {
      // return voltsSignal.getValue();
      return 0.0;
    }

    /**
     * Safely sets a motor voltage with voltage limiting and temperature checks
     *
     * @param talon The motor
     * @param volts The voltage to set
     * @param maxVolts The maximum magnitude for the voltage
     */
    public void setTalonVoltage(TalonFX talon, double volts, double maxVolts) {
      // HardwareUtils.setTalonVoltage(talon, volts, maxVolts, this.tempSignal.getValue());
      HardwareUtils.setTalonVoltage(talon, volts, maxVolts, 0.0);
    }
  }

  /** Status signals for a TalonFX with positional readings */
  public static class TalonSignalsPositional extends TalonSignals {
    private final StatusSignal<Double> posSignal;
    private final StatusSignal<Double> velSignal;

    public TalonSignalsPositional(TalonFX motor) {
      super(motor);
      posSignal = motor.getPosition();
      velSignal = motor.getVelocity();
      posSignal.setUpdateFrequency(100);
      velSignal.setUpdateFrequency(100);
    }

    @Override
    protected void refresh() {
      StatusSignal.refreshAll(posSignal, velSignal);
    }

    public void update(TalonInputsPositional inputs, TalonFX motor) {
      super.update(inputs, motor);
      this.refresh();
      inputs.positionRads = Units.rotationsToRadians(posSignal.getValue());
      inputs.velocityRadsPerSec = Units.rotationsToRadians(velSignal.getValue());
    }
  }
}
