// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
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
import frc.WorBots.util.debug.Logger;

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
  private static final double pivotLengthMeters = Units.inchesToMeters(13.5);

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
  public void update(Vector<N4> angles) {
    base.setLength(angles.get(3, 0));
    pivot.setAngle(Units.radiansToDegrees(-angles.get(0, 0)) + 75);

    /**
     * Some notes: The elevator in use is a contunuous elevator, meaning the parts rise in a linear
     * fashion. Basically you can take the % extended of the whole, and just change it to the range
     * of the first or second carriage. Need the numbers first.
     *
     * <p>The first value is the pivot angle in rads, second one is the first carriage position, the
     * third value is the second carriage position, fourth is the total calculated elevator height
     */

    // 3D logging
    var zeroPose = new Pose3d();
    var baseRotatedPose =
        new Pose3d(
            new Translation3d(0.0, 0.0, 0.0),
            new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0));
    var stageOnePose =
        baseRotatedPose.plus(
            new Transform3d(0.0, 0.0, (0.117 + angles.get(3, 0)), new Rotation3d()));
    // var stageOnePose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    var stageTwoPose =
        stageOnePose.plus(
            new Transform3d(
                new Translation3d(0.0, 0.0, Units.inchesToMeters(1)), new Rotation3d()));
    var pivotpose =
        stageOnePose.plus(
            new Transform3d(0.0, 0.0, 0.51, new Rotation3d(0.0, -angles.get(0, 0), 0.0)));
    Logger.getInstance().setSuperstructurePoses3d(stageOnePose, stageTwoPose, pivotpose);
  }
}
