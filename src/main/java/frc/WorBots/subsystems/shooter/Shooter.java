package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.StatusPage;

public class Shooter extends SubsystemBase {// 532 rpm/v
  private ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();

  private PIDController topFlywheelController;
  private PIDController bottomFlywheelController;
  private SimpleMotorFeedforward topFlywheelFeedForward;
  private SimpleMotorFeedforward bottomFlywheelFeedforward;

  public Shooter(ShooterIO io) {
    this.io = io;

    if (!Constants.getSim()) {
      topFlywheelController = new PIDController(0, 0, 0);
      bottomFlywheelController = new PIDController(0, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    } else {
      topFlywheelController = new PIDController(0, 0, 0);
      bottomFlywheelController = new PIDController(0, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    }

    StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
