// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.WorBots.util.*;

/** The visualizer class of the superstrucutre, logs a Mechanism2d, as well as Pose3d's. */
public class SuperstructureVisualizer {
  private Mechanism2d superstructure;
  private MechanismRoot2d fromGround;
  private MechanismLigament2d base;
  private MechanismLigament2d pivot;

  private static final double zeroPositionToGroundMeters = Units.inchesToMeters(5.250);
  private static final double elevatorAngleDegrees = (105);
  private static final double elevatorPositionX =
      Units.inchesToMeters(13.293); // From center of elevator to edge of front frame - not intake;
  private static final double elevatorFrameLengthAngledMeters = Units.inchesToMeters(18.750);
  private static final double pivotLengthMeters = 0.25;

  /**
   * Constructs a new instance of the visualizer.
   *
   * @param name The name in NT.
   */
  public SuperstructureVisualizer(String name) {
    superstructure = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(36));
    fromGround =
        superstructure.getRoot("fromGround", elevatorPositionX, zeroPositionToGroundMeters);
    base =
        fromGround.append(
            new MechanismLigament2d(name, zeroPositionToGroundMeters, elevatorAngleDegrees));
    pivot = base.append(new MechanismLigament2d("Pivot", pivotLengthMeters, (75)));
    SmartDashboard.putData(name, superstructure);
  }

  /**
   * The function that updates the current visualizer.
   *
   * @param angles The current position of the superstructure.
   */
  public void update(Vector<N2> angles) {
    base.setLength(angles.get(0, 0));
    pivot.setAngle(Units.radiansToDegrees(-angles.get(1, 0)) + 75);

    // 3D logging
    var stageOnePose = new Pose3d(0.0, 0.0, 0.25, new Rotation3d(0.0, 0.0, 0.0));
    var stageTwoPose =
        new Pose3d(
            Units.inchesToMeters(13) - elevatorPositionX,
            0.0,
            zeroPositionToGroundMeters,
            new Rotation3d());
    var pivotpose =
        new Pose3d(
            stageTwoPose.getX(),
            stageTwoPose.getY(),
            stageTwoPose.getZ(),
            new Rotation3d(angles.get(1, 0), 0.0, 0.0));
    Logger.getInstance().setSuperstructurePoses3d(stageOnePose, stageTwoPose, pivotpose);
  }
}
