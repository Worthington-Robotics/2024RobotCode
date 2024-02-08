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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Utility functions for working with hardware devices */
public class HardwareUtils {
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

  /** Base inputs for a TalonFX */
  public static class TalonInputs {
    protected final NetworkTable table;
    private final DoublePublisher voltsPub;
    private final DoublePublisher currentPub;
    private final DoublePublisher tempPub;
    private final BooleanPublisher connectedPub;

    public double appliedPowerVolts = 0.0;
    public double currentDrawAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean isConnected = false;

    public TalonInputs(String table, String subtable) {
      this.table = NetworkTableInstance.getDefault().getTable(table).getSubTable(subtable);
      voltsPub = this.table.getDoubleTopic("Applied Voltage").publish();
      currentPub = this.table.getDoubleTopic("Current Draw").publish();
      tempPub = this.table.getDoubleTopic("Temp Celsius").publish();
      connectedPub = this.table.getBooleanTopic("Connected").publish();
    }

    public void publish() {
      voltsPub.set(appliedPowerVolts);
      currentPub.set(currentDrawAmps);
      tempPub.set(temperatureCelsius);
      connectedPub.set(isConnected);
    }
  }

  /** Inputs for a TalonFX with position and velocity readings */
  public static class TalonInputsPositional extends TalonInputs {
    private final DoublePublisher posPub;
    private final DoublePublisher velPub;

    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;

    public TalonInputsPositional(String table, String subtable) {
      super(table, subtable);
      posPub = this.table.getDoubleTopic("Position").publish();
      velPub = this.table.getDoubleTopic("Velocity").publish();
    }

    public void publish() {
      posPub.set(positionRads);
      velPub.set(velocityRadsPerSec);
    }
  }

  /** Base status signals for a TalonFX */
  public abstract static class TalonSignals {
    private final StatusSignal<Double> tempSignal;
    private final StatusSignal<Double> voltsSignal;
    private final StatusSignal<Double> currentSignal;

    public TalonSignals(TalonFX motor) {
      tempSignal = motor.getDeviceTemp();
      voltsSignal = motor.getSupplyVoltage();
      currentSignal = motor.getSupplyCurrent();

      tempSignal.setUpdateFrequency(10);
      voltsSignal.setUpdateFrequency(100);
      currentSignal.setUpdateFrequency(100);
    }

    public void refresh() {
      StatusSignal.refreshAll(tempSignal, voltsSignal, currentSignal);
    }

    public void update(TalonInputs inputs, TalonFX motor) {
      refresh();
      inputs.temperatureCelsius = tempSignal.getValue();
      inputs.appliedPowerVolts = voltsSignal.getValue() * motor.get();
      inputs.currentDrawAmps = currentSignal.getValue();
      inputs.isConnected = motor.isAlive();
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
    public void refresh() {
      super.refresh();
      StatusSignal.refreshAll(posSignal, velSignal);
    }

    public void update(TalonInputsPositional inputs, TalonFX motor) {
      update(inputs, motor);
      inputs.positionRads = Units.rotationsToRadians(posSignal.getValue());
      inputs.velocityRadsPerSec = Units.rotationsToRadians(velSignal.getValue());
    }
  }
}