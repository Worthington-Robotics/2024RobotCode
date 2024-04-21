// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.util.StateMachine.State;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/** Global simulator for robot functions */
public class RobotSimulator {
  private static RobotSimulator instance = new RobotSimulator();

  public static RobotSimulator getInstance() {
    return instance;
  }

  /** Whether simulation is enabled */
  private final boolean isEnabled;

  /** The state of indexing the game piece in the system */
  private final StateMachine<RobotSimulator> indexState;

  // Data interfaces
  private Superstructure superstructure;
  private Supplier<Pose2d> drivePose = () -> new Pose2d();
  private double intakeVolts = 0.0;
  private double feederVolts = 0.0;
  private double shooterVolts = 0.0;

  private RobotSimulator() {
    isEnabled = RobotBase.isSimulation();
    if (isEnabled) {
      indexState = new StateMachine<>(emptyState);
    } else {
      indexState = null;
    }
  }

  public void setSuperstructure(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  public void setDrivePoseInterface(Supplier<Pose2d> supplier) {
    this.drivePose = supplier;
  }

  public void setIntakeVolts(double volts) {
    this.intakeVolts = volts;
  }

  public void setFeederVolts(double volts) {
    this.feederVolts = volts;
  }

  public void setShooterVolts(double volts) {
    this.shooterVolts = volts;
  }

  /** Periodic function for the simulator */
  public void periodic() {
    if (!isEnabled) {
      return;
    }

    indexState.run(this);
    SmartDashboard.putString("Simulator Index State", indexState.getState().getName());
  }

  /** Loads a game piece into the robot at the shooter, for the beginning of auto */
  public void loadGamePiece() {
    if (!isEnabled) {
      return;
    }

    shooterState.setPosition(0.8);
    indexState.setState(shooterState);
  }

  /** Checks if the intake TOF is triggered in the simulation */
  public boolean isIntakeTofTriggered() {
    if (!isEnabled) {
      return false;
    }

    return (indexState.isInState(intakeState.getClass()) && intakeState.atTof());
  }

  /** Checks if the shooter TOF is triggered in the simulation */
  public boolean isShooterTofTriggered() {
    if (!isEnabled) {
      return false;
    }

    return (indexState.isInState(shooterState.getClass()) && shooterState.atTof());
  }

  /** State for when there is no note in the system */
  private final class EmptyState extends State<RobotSimulator> {
    public Optional<State<RobotSimulator>> run(RobotSimulator sim) {
      // If we are intaking
      if (sim.intakeVolts > 0) {
        if (DriverStation.isAutonomousEnabled()) {
          // Check if we are near a floor game piece during auto simulation
          final Pose2d pose = sim.drivePose.get();
          final double threshold = Units.inchesToMeters(20);

          List<Translation2d> piecePoses = new ArrayList<>();
          for (Translation2d piece : FieldConstants.GamePieces.wingPieces) {
            piecePoses.add(piece);
            piecePoses.add(AllianceFlipUtil.apply(piece));
          }
          for (Translation2d piece : FieldConstants.GamePieces.centerPieces) {
            piecePoses.add(piece);
            piecePoses.add(AllianceFlipUtil.apply(piece));
          }

          for (Translation2d piecePose : piecePoses) {
            if (GeomUtil.isTranslation2dNear(pose.getTranslation(), piecePose, threshold)) {
              intakeState.setPosition(0.0);
              return Optional.of(intakeState);
            }
          }
        } else {
          // Assume that intake was successful in teleop
          intakeState.setPosition(0.0);
          return Optional.of(intakeState);
        }
      }

      return Optional.empty();
    }

    public String getName() {
      return "Empty";
    }
  }

  private final EmptyState emptyState = new EmptyState();

  /** State for when there is a note in the intake */
  private final class IntakeState extends State<RobotSimulator> {
    private double position = 0.0;

    public Optional<State<RobotSimulator>> run(RobotSimulator sim) {
      position += intakeVolts / 90.0;
      SmartDashboard.putNumber("Simulator Index Position", position);
      // Spit
      if (position < 0.0) {
        return Optional.of(emptyState);
      }
      if (position >= 0.9) {
        // Handoff to shooter
        if (superstructure.inHandoff()) {
          shooterState.setPosition(0.0);
          return Optional.of(shooterState);
        }
      }
      if (position > 1.0) {
        // Spit out top of intake
        return Optional.of(emptyState);
      }
      return Optional.empty();
    }

    public void setPosition(double position) {
      this.position = position;
    }

    public boolean atTof() {
      return position >= 0.8;
    }

    public String getName() {
      return "Intake";
    }
  }

  private final IntakeState intakeState = new IntakeState();

  /** State for when there is a note in the shooter */
  private final class ShooterState extends State<RobotSimulator> {
    private double position = 0.0;

    public Optional<State<RobotSimulator>> run(RobotSimulator sim) {
      position += feederVolts / 14.0;
      SmartDashboard.putNumber("Simulator Index Position", position);
      // Spit
      if (position < 0.0) {
        // Handoff to intake
        if (superstructure.inHandoff()) {
          intakeState.setPosition(1.0);
          return Optional.of(intakeState);
        } else {
          // Spit out back of shooter
          return Optional.of(emptyState);
        }
      }
      if (position > 1.0) {
        // Shoot
        if (sim.shooterVolts > 0.0) {
          return Optional.of(emptyState);
        }
      }
      return Optional.empty();
    }

    public void setPosition(double position) {
      this.position = position;
    }

    public boolean atTof() {
      return position >= 0.8;
    }

    public String getName() {
      return "Shooter";
    }
  }

  private final ShooterState shooterState = new ShooterState();
}
