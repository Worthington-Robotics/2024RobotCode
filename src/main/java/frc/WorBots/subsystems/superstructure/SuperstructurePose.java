// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public record SuperstructurePose(Vector<N2> vectorPose) {
  public static enum Preset {
    HOME(new SuperstructurePose(VecBuilder.fill(0, 0))),
    AMP(new SuperstructurePose(VecBuilder.fill(0.0, 0.0))),
    CLIMB(new SuperstructurePose(VecBuilder.fill(0.4, 0.0)));

    public SuperstructurePose pose;

    private Preset(SuperstructurePose pose) {
      this.pose = pose;
    }

    public SuperstructurePose getPose() {
      return pose;
    }

    public Vector<N2> getVecPose() {
      return pose.vectorPose();
    }
  }
}
