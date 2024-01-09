package frc.WorBots.subsystems.vision;

public interface VisionIO {
  public static class VisionIOInputs {
    public double[][] frames = new double[][] {};
    public double timestamps[] = new double[] {};
    public double fps = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {
  }
}