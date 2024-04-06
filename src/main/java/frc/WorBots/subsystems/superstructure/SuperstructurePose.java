// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

/** This class is a simple record of all of the poses that the Superstructure can be in */
public record SuperstructurePose(Vector<N2> vectorPose) {
  public static enum Preset {
    /** Absolute zero position */
    HOME(new SuperstructurePose(VecBuilder.fill(0, 0))),
    /** Pose for scoring in the amp */
    AMP(
        new SuperstructurePose(
            VecBuilder.fill(0.06 + Units.inchesToMeters(1), 1.86 + Units.degreesToRadians(2)))),
    /** Handoff position from intake to shooter */
    HANDOFF(new SuperstructurePose(VecBuilder.fill(0.0, 0.277))),
    /** Stow position for going under the stage. Not quite at zero so that it is faster to reach */
    STOW(new SuperstructurePose(VecBuilder.fill(0.0, 0.1))),
    /** Setpoint shot for the speaker right up against the subwoofer */
    SUBWOOFER_SHOOT(
        new SuperstructurePose(VecBuilder.fill(0.0, 0.498 - Units.degreesToRadians(2)))),
    /** Wing pass over the stage */
    WING_PASS(new SuperstructurePose(VecBuilder.fill(0.0, 0.720))),
    /** Wing pass over the stage */
    STRAIGHT_PASS(
        new SuperstructurePose(VecBuilder.fill(0.0, Superstructure.PIVOT_MAX_ANGLE * 0.85)));

    public SuperstructurePose pose;

    private Preset(SuperstructurePose pose) {
      this.pose = pose;
    }

    /**
     * Get the pose of the preset
     *
     * @return The pose of the preset
     */
    public SuperstructurePose getPose() {
      return pose;
    }

    /**
     * Get the vector pose of the preset
     *
     * @return The vector for the preset's pose
     */
    public Vector<N2> getVecPose() {
      return pose.vectorPose();
    }

    /**
     * Get the elevator component of the pose
     *
     * @return The elevator component
     */
    public double getElevator() {
      return getVecPose().get(0, 0);
    }

    /**
     * Get the pivot component of the pose
     *
     * @return The pivot component
     */
    public double getPivot() {
      return getVecPose().get(1, 0);
    }
  }
}
