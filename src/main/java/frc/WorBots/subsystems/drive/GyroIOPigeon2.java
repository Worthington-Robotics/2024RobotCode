// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.WorBots.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  private final StatusSignal<Double> tempSignal;
  private final StatusSignal<Double> pitchSignal;
  private final StatusSignal<Double> rollSignal;
  private final StatusSignal<Double> yawSignal;
  private final StatusSignal<Double> pitchVelSignal;
  private final StatusSignal<Double> rollVelSignal;
  private final StatusSignal<Double> yawVelSignal;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(0, Constants.SWERVE_CAN_BUS);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    tempSignal = pigeon.getTemperature();
    pitchSignal = pigeon.getPitch();
    rollSignal = pigeon.getRoll();
    yawSignal = pigeon.getYaw();
    pitchVelSignal = pigeon.getAngularVelocityYDevice();
    rollVelSignal = pigeon.getAngularVelocityXWorld();
    yawVelSignal = pigeon.getAngularVelocityZDevice();
    StatusSignal.setUpdateFrequencyForAll(
        100,
        tempSignal,
        pitchSignal,
        rollSignal,
        yawSignal,
        pitchVelSignal,
        rollVelSignal,
        yawVelSignal);
    pigeon.optimizeBusUtilization();
    pigeon.reset();
  }

  public void updateInputs(GyroIOInputs inputs) {
    tempSignal.refresh();
    pitchSignal.refresh();
    rollSignal.refresh();
    yawSignal.refresh();
    pitchVelSignal.refresh();
    rollVelSignal.refresh();
    yawVelSignal.refresh();

    inputs.connected = tempSignal.getValue() != 0.0;
    inputs.rollPositionRad = Units.degreesToRadians(rollSignal.getValue());
    inputs.pitchPositionRad = Units.degreesToRadians(pitchSignal.getValue());
    inputs.yawPositionRad = Units.degreesToRadians(yawSignal.getValue());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pitchVelSignal.getValue());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelSignal.getValue());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelSignal.getValue());
  }

  public void resetHeading() {
    pigeon.reset();
  }
}
