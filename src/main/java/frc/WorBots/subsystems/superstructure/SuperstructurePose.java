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

/** This class is a simple record of all of the poses that the Superstructure can be in */
public record SuperstructurePose(Vector<N2> vectorPose) {
  public static enum Preset {
    HOME(new SuperstructurePose(VecBuilder.fill(0, 0))),
    // AMP(new SuperstructurePose(VecBuilder.fill(0.26, 2))),
    AMP2(new SuperstructurePose(VecBuilder.fill(0.05924, 2.23))),
    CENTER(new SuperstructurePose(VecBuilder.fill(0.15, 0.0))),
    HANDOFF(new SuperstructurePose(VecBuilder.fill(0.0, 0.277))),
    PIVOTTOTOP(new SuperstructurePose(VecBuilder.fill(0.0, 1.8))),
    STOW(new SuperstructurePose(VecBuilder.fill(0.0, 0.1))),
    TRAP(new SuperstructurePose(VecBuilder.fill(0.25, 0.2421))),
    START_CLIMB(new SuperstructurePose(VecBuilder.fill(0.26, Superstructure.pivotMaxAngle / 2))),
    CLIMB(new SuperstructurePose(VecBuilder.fill(0.035, Superstructure.pivotMaxAngle / 2))),
    SLIDE(new SuperstructurePose(VecBuilder.fill(0.0, Superstructure.pivotMaxAngle)));

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
