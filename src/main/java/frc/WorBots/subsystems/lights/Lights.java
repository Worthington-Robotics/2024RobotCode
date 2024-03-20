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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  /** The number of LEDs */
  public static final int LIGHT_COUNT = 27;

  /** The ID of the LEDs */
  private static final int LIGHTS_ID = 9;

  /** The time to wait when targeting before showing green */
  private static final double TARGETING_TIME = 0.5;

  private final AddressableLED leds;
  private final AddressableLEDBuffer io;
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
    Disabled,
    Claire,
    Shooting,
    Delivery,
    RedBlue,
    Indicator,
    Elevator,
    Field,
    ShootReady,
    PitTest,
  }

  /** The lights subsystem, which is rather pretty. */
  private Lights() {
    // Initialize lights and buffer
    leds = new AddressableLED(LIGHTS_ID);
    io = new AddressableLEDBuffer(LIGHT_COUNT);
    leds.setLength(LIGHT_COUNT);
    leds.start();

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
        rainbow(50.0, 1.5);
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
      case Disabled:
        wave(Color.kBlack, Color.kBlue, 25.0, 2.0, 0.4);
        break;
      case Claire:
        wave(Color.kPurple, Color.kBlack, 25.0, 2.0, 0.4);
        break;
      case Shooting:
        shooting();
        break;
      case Delivery:
        delivery();
        break;
      case RedBlue:
        // final double time = Timer.getFPGATimestamp();
        // var red = (time % 1.0 < 0.5) ? new Color(0.4, 0.0, 0.0) : Color.kRed;
        // var blue = (time % 1.0 > 0.5) ? new Color(0.0, 0.0, 0.4) : Color.kBlue;
        var red = Color.kRed;
        var blue = Color.kBlue;
        wave(blue, red, 14.0, 1.2, 0.3);
        break;
      case Indicator:
        solid(Color.kRed);
        break;
      case Elevator:
        elevator();
        break;
      case Field:
        final boolean hasGamePieceTop =
            hasGamePieceTopDebouncer.calculate(this.hasGamePieceTop.get());
        if (hasGamePieceTop) {
          solid(Color.kRed);
        } else {
          worbotsBounce();
          // wave(Color.kBlue, Color.kRed, 14.0, 1.2, 0.3);
        }
        break;
      case ShootReady:
        final boolean atShootSetpoint = this.atShootSetpoint.get();
        final Color color = atShootSetpoint ? Color.kGreen : Color.kOrangeRed;
        solid(color);
        break;
      case PitTest:
        pitTest();
        break;
    }

    // Dim all the LEDs
    for (int i = 0; i < LIGHT_COUNT; i++) {
      final Color color = io.getLED(i);
      final double factor = 0.9;
      final Color dimmed = new Color(color.red * factor, color.green * factor, color.blue * factor);
      setLED(i, dimmed);
    }

    // Set the LEDs
    leds.setData(io);
  }

  public void setTargetedSupplier(Supplier<Boolean> supplier) {
    this.isTargeted = supplier;
  }

  /**
   * Sets all the lights to a single color
   *
   * @param color The color
   */
  private void solid(Color color) {
    solid(color, 1.0);
  }

  /**
   * Sets a portion of the lights to a single color
   *
   * @param color The color
   * @param percent The portion (0-1) to set
   */
  private void solid(Color color, double percent) {
    final int count = (int) (LIGHT_COUNT * percent);
    for (int i = 0; i < count; i++) {
      setLED(i, color);
    }
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((TimeCache.getInstance().get() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < LIGHT_COUNT; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= 0) {
        setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration, double waveExponent) {
    double x = (1 - ((TimeCache.getInstance().get() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < LIGHT_COUNT; i++) {
      x += xDiffPerLed;
      if (i >= 0) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        setLED(i, new Color(red, green, blue));
      }
    }
  }

  public void bounce(Color c1, double cycleLength, double duration, double waveExponent) {
    double x = (Math.sin((TimeCache.getInstance().get() % (Math.PI * 2))) + 1) / 2;
    double xDiffPerLed = cycleLength;
    for (int i = 0; i < LIGHT_COUNT; i++) {
      x -= xDiffPerLed;
      if (i >= 0) {
        double red = (c1.red * (x));
        setLED(i, new Color(red, red, red));
      }
    }
  }

  private void status() {
    final int len = MathUtil.clamp(StatusPage.ALL_SYSTEMS.length, 0, LIGHT_COUNT);
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
        setLED(i, Color.kGreen);
      } else {
        if (isWarning) {
          setLED(i, Color.kOrangeRed);
        } else {
          setLED(i, Color.kRed);
        }
      }
    }

    // Clear additional lights
    for (int i = StatusPage.ALL_SYSTEMS.length; i < LIGHT_COUNT; i++) {
      setLED(i, Color.kBlack);
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
      solid(color1);
    } else {
      solid(color2);
    }
  }

  private void worbotsBounce() {
    solid(Color.kBlack);

    final double time = 1.0;
    final double percent =
        Math.pow(
            (Math.sin(TimeCache.getInstance().get() % time / time * 2.0 * Math.PI) + 1.0) / 2.0,
            0.8);
    final double portion = percent * 0.5;
    final int width = 6;

    final int index0 = (int) (portion * (LIGHT_COUNT - width * 1.5));
    final int index1 = index0 + width;
    final int index2 = LIGHT_COUNT - index0;
    final int index3 = index2 - width;

    for (int i = index0; i < index1; i++) {
      setLED(i, Color.kRed);
      if (i > LIGHT_COUNT * 0.4) {
        setLED(i, Color.kWhite);
      } else if (i < 4) {
        setLED(i, Color.kDarkRed);
      }
    }

    for (int i = index3; i < index2; i++) {
      setLED(i, Color.kBlue);
      if (i < LIGHT_COUNT * 0.58) {
        setLED(i, Color.kWhite);
      } else if (i >= LIGHT_COUNT - 4) {
        setLED(i, Color.kDarkBlue);
      }
    }
  }

  private void alliance() {
    final var alliance = AllianceCache.getInstance().get();
    final boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    wave(isRed ? Color.kRed : Color.kBlue, Color.kBlack, 25.0, 2.0, 0.4);
  }

  private void matchTime() {
    final double autoSeconds = 15.0;
    final double teleopSeconds = 135.0;
    final double remaining = MatchTime.getInstance().getTime();
    double ratio = remaining;
    if (DriverStation.isAutonomousEnabled()) {
      ratio /= autoSeconds;
    } else if (DriverStation.isTeleopEnabled()) {
      ratio /= teleopSeconds;
    } else {
      ratio = 1.0;
    }
    final boolean isLittleTimeLeft = ratio <= 0.50;
    final boolean isVeryLittleTimeLeft = ratio <= 0.20;
    final int lightCount = (int) (ratio * LIGHT_COUNT);
    for (int i = 0; i < LIGHT_COUNT; i++) {
      // Put a brighter light at the very end
      final int value = (i == lightCount) ? 255 : 180;

      if (i <= lightCount) {
        if (isVeryLittleTimeLeft) {
          setHSV(i, 0, 255, value);
        } else if (isLittleTimeLeft) {
          setHSV(i, 30, 255, value);
        } else {
          setHSV(i, 0, 0, value);
        }
      } else {
        setHSV(i, 0, 0, 0);
      }
    }
  }

  private void shooting() {
    if (!targetingTimer.hasElapsed(TARGETING_TIME)) {
      solid(Color.kOrange);
      return;
    }

    final Pose2d pose = drivePoseSupplier.get();
    final ChassisSpeeds speeds = driveSpeedsSupplier.get();
    final boolean targeted = isTargeted.get();

    final ShotConfidence confidence = ShooterMath.calculateConfidence(pose, speeds);

    if (!targeted || confidence.equals(ShotConfidence.LOW)) {
      solid(Color.kRed);
      return;
    }

    if (confidence.equals(ShotConfidence.MEDIUM)) {
      solid(Color.kOrange);
    } else if (confidence.equals(ShotConfidence.HIGH)) {
      solid(Color.kGreen);
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
      solid(Color.kOrangeRed);
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
    solid(Color.kBlack);
    solid(Color.kOrangeRed, percent);
  }

  private void pitTest() {
    if (pitTestFlashing) {
      blink(Color.kOrangeRed, Color.kBlack, 0.25, TimeCache.getInstance().get());
    }
    solid(Color.kGreen, pitTestStep / 10.0);
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

  public void setPitTestStep(int step) {
    this.pitTestStep = step;
  }

  public void setPitTestFlashing(boolean flashing) {
    this.pitTestFlashing = flashing;
  }

  /**
   * Wrapper function that sets an LED
   *
   * @param index The index of the LED
   * @param color The desired color
   */
  private void setLED(int index, Color color) {
    if (index >= LIGHT_COUNT) {
      return;
    }

    io.setLED(index, color);
  }

  private void setHSV(int index, int h, int s, int v) {
    if (index >= LIGHT_COUNT) {
      return;
    }

    io.setHSV(index, index, index, index);
  }
}
