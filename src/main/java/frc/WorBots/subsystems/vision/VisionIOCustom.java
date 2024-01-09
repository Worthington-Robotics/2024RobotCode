package frc.WorBots.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class VisionIOCustom implements VisionIO {
  private NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
  private NetworkTable table;
  private NetworkTable subTable;
  private DoubleArraySubscriber data;
  private DoubleSubscriber subFps;
  private int index;

  public VisionIOCustom(int index) {
    table = defaultInstance.getTable("module" + index);
    subTable = table.getSubTable("output");
    data = subTable.getDoubleArrayTopic("data").subscribe(new double[] {}, PubSubOption.keepDuplicates(true),
        PubSubOption.sendAll(true));
    subFps = subTable.getDoubleTopic("fps").subscribe(0.0, PubSubOption.keepDuplicates(true),
        PubSubOption.sendAll(true));
    this.index = index;
  }

  public void updateInputs(VisionIOInputs inputs) {
    var frame = data.readQueue();
    int length = frame.length;

    inputs.timestamps = new double[length];
    inputs.frames = new double[length][];

    for (int i = 0; i < frame.length; i++) {
      inputs.timestamps[i] = frame[i].timestamp / 1000000.0;
      if (index == 1) {
        System.out.println(inputs.timestamps[i]);
      }
      inputs.frames[i] = frame[i].value;
    }
    
    inputs.fps = subFps.get();
  }
}