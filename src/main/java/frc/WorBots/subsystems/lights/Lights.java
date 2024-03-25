// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.lights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.lights.LightsUtil.ColorSequence;
import frc.WorBots.subsystems.lights.LightsUtil.Lava;
import frc.WorBots.util.MatchTime;
import frc.WorBots.util.StateMachine;
import frc.WorBots.util.StateMachine.State;
import frc.WorBots.util.StateMachine.StateTransition;
import frc.WorBots.util.cache.Cache.AllianceCache;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.ShooterMath;
import frc.WorBots.util.math.ShooterMath.ShotConfidence;
import java.util.Optional;
import java.util.function.Supplier;

public class Lights extends SubsystemBase {
  private static Lights instance = new Lights();

  public static Lights getInstance() {
    return instance;
  }

  // Constants
  /** The time to wait when targeting before showing green */
  private static final double TARGETING_TIME = 0.5;

  /** The number of steps in the pit test */
  private static final int PIT_TEST_STEP_COUNT = 10;

  private static final ColorSequence LAVA_COLORS =
      new ColorSequence(Color.kBlack, Color.kDarkBlue, Color.kBlue, Color.kRed);

  private static final ColorSequence FLAME_COLORS =
      new ColorSequence(
          Color.kCadetBlue,
          Color.kBlue,
          Color.kIndigo,
          Color.kRed,
          Color.kRed,
          Color.kRed,
          Color.kBlack);

  private final LightsIO io;
  private LightsMode mode = LightsMode.RedBlue;

  private final IntegerSubscriber setModeSub;
  private final IntegerPublisher setModePub;

  // Mode-specific state variables

  /** Timer for targeting mode */
  private final Timer targetingTimer = new Timer();

  /** Debouncer for the bottom ToF */
  private final Debouncer hasGamePieceBottomDebouncer = new Debouncer(0.10);

  /** Debouncer for the top ToF */
  private final Debouncer hasGamePieceTopDebouncer = new Debouncer(0.20);

  /** State of the delivery mode */
  private final StateMachine<Lights> deliveryState;

  /** Timestamp when intaking started */
  private double startOfIntakeTime = 0.0;

  /** Lava mode state */
  private final Lava lava = new Lava();

  // Data interfaces
  private Supplier<Boolean> isTargeted = () -> false;
  private Supplier<Pose2d> drivePoseSupplier = () -> new Pose2d();
  private Supplier<ChassisSpeeds> driveSpeedsSupplier = () -> new ChassisSpeeds();
  private Supplier<Boolean> inHandoff = () -> false;
  private Supplier<Boolean> inStow = () -> false;
  private Supplier<Boolean> atShootSetpoint = () -> false;
  private Supplier<Boolean> hasGamePieceBottom = () -> false;
  private Supplier<Boolean> hasGamePieceTop = () -> false;
  private Supplier<Double> elevatorPercentageRaised = () -> 0.0;

  /** Step in the pit test */
  private int pitTestStep = 0;

  /** Whether the pit test mode is flashing */
  private boolean pitTestFlashing = false;

  /** Modes for light control */
  public enum LightsMode {
    Rainbow,
    Status,
    Alliance,
    MatchTime,
    Claire,
    Shooting,
    Delivery,
    RedBlue,
    Indicator,
    Elevator,
    Field,
    ShootReady,
    PitTest,
    Lava,
    Flame,
  }

  /** The lights subsystem, which is rather pretty. */
  private Lights() {
    io = new LightsIO();

    // Initialize NT
    final var table =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Lights");
    setModeSub = table.getIntegerTopic("Set Mode").subscribe(-1);
    setModePub = table.getIntegerTopic("Mode Number").publish();
    StatusPage.reportStatus(StatusPage.LIGHTS_SUBSYSTEM, true);

    // Initialize states
    deliveryState = new StateMachine<>(defaultState);
  }

  public void periodic() {
    // Read from mode setting publisher
    for (long val : setModeSub.readQueueValues()) {
      mode = LightsMode.values()[(int) val];
    }

    setModePub.set(mode.ordinal());
    SmartDashboard.putString("Lights/Mode", mode.toString());

    // Run the current mode
    switch (mode) {
      case Rainbow:
        LightsUtil.rainbow(io, 50.0, 1.5);
        break;
      case Status:
        status();
        break;
      case Alliance:
        alliance();
        break;
      case MatchTime:
        matchTime();
        break;
      case Claire:
        LightsUtil.wave(io, new ColorSequence(Color.kPurple, Color.kBlack), 25.0, 2.0, 0.4);
        break;
      case Shooting:
        shooting();
        break;
      case Delivery:
        delivery();
        break;
      case RedBlue:
        LightsUtil.wave(
            io, new ColorSequence(Color.kRed, Color.kBlack, Color.kBlue), 14.0, 1.2, 0.3);
        break;
      case Indicator:
        LightsUtil.solid(io, Color.kRed);
        break;
      case Elevator:
        elevator();
        break;
      case Field:
        final boolean hasGamePieceTop =
            hasGamePieceTopDebouncer.calculate(this.hasGamePieceTop.get());
        if (hasGamePieceTop) {
          LightsUtil.solid(io, Color.kRed);
        } else {
          worbotsBounce();
          // wave(Color.kBlue, Color.kRed, 14.0, 1.2, 0.3);
        }
        break;
      case ShootReady:
        final boolean atShootSetpoint = this.atShootSetpoint.get();
        final Color color = atShootSetpoint ? Color.kGreen : Color.kOrangeRed;
        LightsUtil.solid(io, color);
        break;
      case PitTest:
        pitTest();
        break;
      case Lava:
        lava.run(io, LAVA_COLORS);
        break;
      case Flame:
        LightsUtil.flame(io, 0.95, FLAME_COLORS);
        break;
    }

    io.periodic();
  }

  public void setTargetedSupplier(Supplier<Boolean> supplier) {
    this.isTargeted = supplier;
  }

  private void status() {
    final int len = MathUtil.clamp(StatusPage.ALL_SYSTEMS.length, 0, io.getCount());
    for (int i = 0; i < len; i++) {
      final String system = StatusPage.ALL_SYSTEMS[i];
      // Some statuses are not necessarily errors and
      // we want to display them as yellow
      boolean isWarning = false;
      if (system.equals(StatusPage.LAUNCHPAD) || system.equals(StatusPage.IDEAL_BATTERY)) {
        isWarning = true;
      }
      final boolean status = StatusPage.getStatus(system);
      if (status) {
        io.setLED(i, Color.kGreen);
      } else {
        if (isWarning) {
          io.setLED(i, Color.kOrangeRed);
        } else {
          io.setLED(i, Color.kRed);
        }
      }
    }

    // Clear additional lights
    for (int i = StatusPage.ALL_SYSTEMS.length; i < io.getCount(); i++) {
      io.setLED(i, Color.kBlack);
    }
  }

  /**
   * Blinks the LEDs in a two-color pattern
   *
   * @param color1 The first color, comes first in the blink
   * @param color2 The second color
   * @param interval The interval in seconds between each color change
   * @param time The current time or relative time from a timer in seconds
   */
  private void blink(Color color1, Color color2, double interval, double time) {
    if (time % (interval * 2.0) <= interval) {
      LightsUtil.solid(io, color1);
    } else {
      LightsUtil.solid(io, color2);
    }
  }

  private void worbotsBounce() {
    LightsUtil.solid(io, Color.kBlack);

    final double time = 1.0;
    final double percent =
        Math.pow(
            (Math.sin(TimeCache.getInstance().get() % time / time * 2.0 * Math.PI) + 1.0) / 2.0,
            0.8);
    final double portion = percent * 0.5;
    final int width = 6;

    final int index0 = (int) (portion * (io.getCount() - width * 1.5));
    final int index1 = index0 + width;
    final int index2 = io.getCount() - index0;
    final int index3 = index2 - width;

    for (int i = index0; i < index1; i++) {
      io.setLED(i, Color.kRed);
      if (i > io.getCount() * 0.4) {
        io.setLED(i, Color.kWhite);
      } else if (i < 4) {
        io.setLED(i, Color.kDarkRed);
      }
    }

    for (int i = index3; i < index2; i++) {
      io.setLED(i, Color.kBlue);
      if (i < io.getCount() * 0.58) {
        io.setLED(i, Color.kWhite);
      } else if (i >= io.getCount() - 4) {
        io.setLED(i, Color.kDarkBlue);
      }
    }
  }

  private void alliance() {
    final var alliance = AllianceCache.getInstance().get();
    final boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    final ColorSequence colors = new ColorSequence(isRed ? Color.kRed : Color.kBlue, Color.kBlack);
    LightsUtil.wave(io, colors, 25.0, 2.0, 0.4);
  }

  private void matchTime() {
    final double autoSeconds = 15.0;
    final double teleopSeconds = 135.0;
    final double remaining = MatchTime.getInstance().getTimeRemaining();
    double ratio = remaining;
    if (DriverStation.isAutonomous()) {
      ratio /= autoSeconds;
    } else if (DriverStation.isTeleop()) {
      ratio /= teleopSeconds;
    } else {
      ratio = 1.0;
    }
    final boolean isLittleTimeLeft = ratio <= 0.50;
    final boolean isVeryLittleTimeLeft = ratio <= 0.20;
    final int lightCount = (int) (ratio * io.getCount());
    for (int i = 0; i < io.getCount(); i++) {
      // Put a brighter light at the very end
      final int value = (i == lightCount) ? 255 : 180;

      if (i <= lightCount) {
        if (isVeryLittleTimeLeft) {
          io.setHSV(i, 0, 255, value);
        } else if (isLittleTimeLeft) {
          io.setHSV(i, 25, 255, value);
        } else {
          io.setHSV(i, 0, 0, value);
        }
      } else {
        io.setHSV(i, 0, 0, 0);
      }
    }
  }

  private void shooting() {
    if (!targetingTimer.hasElapsed(TARGETING_TIME)) {
      LightsUtil.solid(io, Color.kOrange);
      return;
    }

    final Pose2d pose = drivePoseSupplier.get();
    final ChassisSpeeds speeds = driveSpeedsSupplier.get();
    final boolean targeted = isTargeted.get();

    final ShotConfidence confidence = ShooterMath.calculateConfidence(pose, speeds);

    if (!targeted || confidence.equals(ShotConfidence.LOW)) {
      LightsUtil.solid(io, Color.kRed);
      return;
    }

    if (confidence.equals(ShotConfidence.MEDIUM)) {
      LightsUtil.solid(io, Color.kOrange);
    } else if (confidence.equals(ShotConfidence.HIGH)) {
      LightsUtil.solid(io, Color.kGreen);
    }
  }

  private void delivery() {
    deliveryState.run(this);
    if (inHandoff.get() && !deliveryState.isInState(startOfIntakeState)) {
      SmartDashboard.putNumber("Fart 2", Timer.getFPGATimestamp());
      blink(
          Color.kWhite,
          Color.kBlack,
          intakeBlinkInterval,
          TimeCache.getInstance().get() - startOfIntakeTime);
    }
    SmartDashboard.putString("Lights/Delivery State", deliveryState.getState().getName());
  }

  private static final double intakeBlinkInterval = 0.28;
  private final Timer intakeTimer = new Timer();

  private class IntakeTransition implements StateTransition<Lights> {
    /** Whether we had a game piece in the bottom in the last period */
    private boolean hadGamePieceBottomBefore = false;

    public Optional<State<Lights>> getTransition(Lights inputs) {
      final boolean hasGamePieceBottom =
          hasGamePieceBottomDebouncer.calculate(Lights.this.hasGamePieceBottom.get());
      if (!hadGamePieceBottomBefore && hasGamePieceBottom) {
        return Optional.of(startOfIntakeState);
      }
      hadGamePieceBottomBefore =
          hasGamePieceBottom && !hasGamePieceTopDebouncer.calculate(hasGamePieceTop.get());
      return Optional.empty();
    }
  }

  private final IntakeTransition intakeTransition = new IntakeTransition();

  private class ReadyTransition implements StateTransition<Lights> {
    public Optional<State<Lights>> getTransition(Lights inputs) {
      if (hasGamePieceTopDebouncer.calculate(hasGamePieceTop.get())) {
        return Optional.of(readyState);
      }
      return Optional.empty();
    }
  }

  private final ReadyTransition readyTransition = new ReadyTransition();

  private class DefaultState extends State<Lights> {
    public DefaultState() {
      super(intakeTransition, readyTransition);
    }

    public String getName() {
      return "Default";
    }

    @Override
    public Optional<State<Lights>> run(Lights input) {
      if (inStow.get()) {
        worbotsBounce();
        return Optional.empty();
      }

      alliance();
      return Optional.empty();
    }
  }

  private final DefaultState defaultState = new DefaultState();

  private class StartOfIntakeState extends State<Lights> {
    public String getName() {
      return "Start of Intake";
    }

    public void initialize() {
      startOfIntakeTime = TimeCache.getInstance().get();
      intakeTimer.restart();
    }

    @Override
    public Optional<State<Lights>> run(Lights input) {
      final double halfInterval = intakeBlinkInterval / 2.0;
      SmartDashboard.putNumber("Fart", TimeCache.getInstance().get() - startOfIntakeTime);
      if (TimeCache.getInstance().get() - startOfIntakeTime >= (halfInterval * 6)) {
        if (hasGamePieceTopDebouncer.calculate(hasGamePieceTop.get())) {
          return Optional.of(readyState);
        }
        return Optional.of(stillInIntakeState);
      }
      blink(
          Color.kGreen,
          Color.kBlack,
          halfInterval,
          TimeCache.getInstance().get() - startOfIntakeTime);
      return Optional.empty();
    }
  }

  private final StartOfIntakeState startOfIntakeState = new StartOfIntakeState();

  private class StillInIntakeState extends State<Lights> {
    public StillInIntakeState() {
      super(readyTransition);
    }

    public String getName() {
      return "Still in Intake";
    }

    @Override
    public Optional<State<Lights>> run(Lights input) {
      if (!hasGamePieceBottomDebouncer.calculate(hasGamePieceBottom.get())) {
        return Optional.of(defaultState);
      }
      // Blink to notify a note is still in the intake
      blink(
          Color.kOrangeRed,
          Color.kBlack,
          intakeBlinkInterval,
          TimeCache.getInstance().get() - startOfIntakeTime);
      return Optional.empty();
    }
  }

  private final StillInIntakeState stillInIntakeState = new StillInIntakeState();

  private class ReadyState extends State<Lights> {
    public ReadyState() {
      super();
    }

    public String getName() {
      return "Ready";
    }

    @Override
    public Optional<State<Lights>> run(Lights input) {
      LightsUtil.solid(io, Color.kOrangeRed);
      if (!hasGamePieceTopDebouncer.calculate(hasGamePieceTop.get())
          && !hasGamePieceBottomDebouncer.calculate(hasGamePieceBottom.get())) {
        return Optional.of(shotState);
      }
      return Optional.empty();
    }
  }

  private final ReadyState readyState = new ReadyState();

  private class ShotState extends State<Lights> {
    private double startTime;

    public String getName() {
      return "Ready";
    }

    public void initialize() {
      startTime = TimeCache.getInstance().get();
    }

    @Override
    public Optional<State<Lights>> run(Lights input) {
      blink(
          Color.kGreen,
          Color.kBlack,
          intakeBlinkInterval,
          TimeCache.getInstance().get() - startTime);
      if (TimeCache.getInstance().get() - startTime >= intakeBlinkInterval * 2.0) {
        return Optional.of(defaultState);
      }
      return Optional.empty();
    }
  }

  private final ShotState shotState = new ShotState();

  private void elevator() {
    final double percent = elevatorPercentageRaised.get();
    LightsUtil.solid(io, Color.kBlack);
    LightsUtil.solid(io, Color.kOrangeRed, percent);
  }

  private void pitTest() {
    if (pitTestFlashing) {
      blink(Color.kOrangeRed, Color.kBlack, 0.25, TimeCache.getInstance().get());
    }
    LightsUtil.solid(io, Color.kGreen, pitTestStep / PIT_TEST_STEP_COUNT);
  }

  /**
   * Sets the mode of the lights
   *
   * @param mode The mode to set
   */
  public void setMode(LightsMode mode) {
    // Reset persistent stuff for some modes if this new mode is different
    if (!mode.equals(this.mode)) {
      if (mode.equals(LightsMode.Shooting)) {
        targetingTimer.restart();
      }
    }
    this.mode = mode;
  }

  /** Sets data interfaces for subsystems */
  public void setDataInterfaces(
      Supplier<Pose2d> drivePoseSupplier,
      Supplier<ChassisSpeeds> driveSpeedsSupplier,
      Supplier<Boolean> inHandoff,
      Supplier<Boolean> inStow,
      Supplier<Boolean> atShootSetpoint,
      Supplier<Boolean> hasGamePieceBottom,
      Supplier<Boolean> hasGamePieceTop,
      Supplier<Double> elevatorPercentageRaised) {
    this.drivePoseSupplier = drivePoseSupplier;
    this.driveSpeedsSupplier = driveSpeedsSupplier;
    this.inHandoff = inHandoff;
    this.inStow = inStow;
    this.atShootSetpoint = atShootSetpoint;
    this.hasGamePieceBottom = hasGamePieceBottom;
    this.hasGamePieceTop = hasGamePieceTop;
    this.elevatorPercentageRaised = elevatorPercentageRaised;
  }

  /** Sets the current step of the pit test in pit test mode */
  public void setPitTestStep(int step) {
    this.pitTestStep = step;
  }

  /** Sets whether the lights are flashing in pit test mode */
  public void setPitTestFlashing(boolean flashing) {
    this.pitTestFlashing = flashing;
  }
}
