// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

public class NoteVisionIONative implements NoteVisionIO {
  private final NoteVisionGRIP grip = new NoteVisionGRIP();
  private final CvSink camera;

  public NoteVisionIONative() {
    camera = CameraServer.getVideo(CameraServer.startAutomaticCapture(0));
  }

  public void updateInputs(NoteVisionIOInputs inputs) {
    Mat mat = new Mat();
    final long result = camera.grabFrame(mat, Constants.ROBOT_PERIOD / 2.0);
    if (result != 0) {
      grip.process(mat);
      final ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
      ArrayList<Double> out = new ArrayList<>();
      for (var contour : contours) {
        final Point[] points = contour.toArray();
        for (Point point : points) {
          out.add(point.x);
          out.add(point.y);
        }
      }
      SmartDashboard.putNumberArray("Contours", out.toArray(new Double[] {}));
    }

    // Flip the sign to make it CCW positive
    // inputs.noteTheta = noteThetaFilter.calculate(-Units.degreesToRadians(noteThetaSub.get()));
    // inputs.notePitch = notePitchFilter.calculate(Units.degreesToRadians(notePitchSub.get()));
    // inputs.noteArea = noteAreaFilter.calculate(Units.degreesToRadians(noteAreaSub.get()));
    // inputs.noteRatio =
    //     notePixelsXFilter.calculate(notePixelsXSub.get())
    //         / notePixelsYFilter.calculate(notePixelsYSub.get());
    // if (Double.isNaN(inputs.noteRatio)) {
    //   inputs.noteRatio = 0.0;
    // }
    // inputs.hasTarget = hasTargetSub.get();
  }
}
