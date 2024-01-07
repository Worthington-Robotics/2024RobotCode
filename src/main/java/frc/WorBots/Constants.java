package frc.WorBots;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final boolean getSim() {
      return RobotBase.isSimulation();
    }

    public class Field {
      public static final double FIELD_LENGTH = 16.54175;
      public static final double FIELD_WIDTH = 8.0137;
    }
}
