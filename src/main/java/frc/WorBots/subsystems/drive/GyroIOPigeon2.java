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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.WorBots.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  private final StatusSignal<Double> yawSignal;
  private final StatusSignal<Double> yawVelSignal;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(0, Constants.SWERVE_CAN_BUS);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    yawSignal = pigeon.getYaw();
    yawVelSignal = pigeon.getAngularVelocityZDevice();
    StatusSignal.setUpdateFrequencyForAll(100, yawSignal, yawVelSignal);
    pigeon.optimizeBusUtilization();
    pigeon.reset();
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = StatusSignal.refreshAll(yawSignal, yawVelSignal).isOK();
    inputs.yawPositionRad = Units.degreesToRadians(yawSignal.getValue());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelSignal.getValue());
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    // pigeon.reset();
    pigeon.setYaw(heading.getDegrees());
  }
}
