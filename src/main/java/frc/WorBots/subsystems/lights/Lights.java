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
import frc.WorBots.util.cache.Cache.AllianceCache;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.ShooterMath;
import frc.WorBots.util.math.ShooterMath.ShotConfidence;
import java.util.function.Supplier;

public class Lights extends SubsystemBase {
  private static Lights instance = new Lights();

  public static Lights getInstance() {
    return instance;
  }

  public static final int LIGHT_COUNT = 27;
  private static final int LIGHTS_ID = 9;
  private final AddressableLED leds;
  private final AddressableLEDBuffer io;
  private LightsMode mode = LightsMode.RedBlue;
  private final IntegerSubscriber setModeSub;
  private final IntegerPublisher setModePub;
  private final Timer timer = new Timer();
  private boolean hasStartedTimer = false;
  private Supplier<Boolean> isTargeted = () -> false;
  // Data interfaces
  private Supplier<Pose2d> drivePoseSupplier = () -> new Pose2d();
  private Supplier<ChassisSpeeds> driveSpeedsSupplier = () -> new ChassisSpeeds();
  private Supplier<Boolean> inHandoff = () -> false;
  private Supplier<Boolean> inStow = () -> false;
  private Supplier<Boolean> atShootSetpoint = () -> false;
  private Supplier<Boolean> hasGamePieceBottom = () -> false;
  private Supplier<Boolean> hasGamePieceTop = () -> false;
  private Supplier<Double> elevatorPercentageRaised = () -> 0.0;
  private int pitTestStep = 0;
  private boolean pitTestFlashing = false;

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
    leds = new AddressableLED(LIGHTS_ID);
    io = new AddressableLEDBuffer(LIGHT_COUNT);
    leds.setLength(LIGHT_COUNT);
    leds.start();

    final var table =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Lights");
    setModeSub = table.getIntegerTopic("Set Mode").subscribe(-1);
    setModePub = table.getIntegerTopic("Mode Number").publish();
    StatusPage.reportStatus(StatusPage.LIGHTS_SUBSYSTEM, true);
  }

  public void periodic() {
    for (long val : setModeSub.readQueueValues()) {
      mode = LightsMode.values()[(int) val];
    }

    setModePub.set(mode.ordinal());

    switch (mode) {
      case Rainbow:
        rainbow(100, 50.0, 1.5);
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
        wave(100, Color.kBlack, Color.kBlue, 25.0, 2.0, 0.4);
        break;
      case Claire:
        wave(100, Color.kPurple, Color.kBlack, 25.0, 2.0, 0.4);
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
        wave(100, blue, red, 14.0, 1.2, 0.3);
        break;
      case Indicator:
        solid(1.0, Color.kRed);
        break;
      case Elevator:
        elevator();
        break;
      case Field:
        final boolean hasGamePieceTop =
            hasGamePieceTopDebouncer.calculate(this.hasGamePieceTop.get());
        if (hasGamePieceTop) {
          solid(1.0, Color.kRed);
        } else {
          wave(100, Color.kBlue, Color.kRed, 14.0, 1.2, 0.3);
        }
        break;
      case ShootReady:
        final boolean atShootSetpoint = this.atShootSetpoint.get();
        final Color color = atShootSetpoint ? Color.kGreen : Color.kOrangeRed;
        solid(1.0, color);
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
      io.setLED(i, dimmed);
    }

    leds.setData(io);
    SmartDashboard.putString("Lights/Mode", mode.toString());
  }

  public void setTargetedSupplier(Supplier<Boolean> supplier) {
    this.isTargeted = supplier;
  }

  private void solid(double percent, Color color) {
    final int count = (int) (LIGHT_COUNT * percent);
    for (int i = 0; i < count; i++) {
      io.setLED(i, color);
    }
  }

  private void rainbow(double percent, double cycleLength, double duration) {
    double x = (1 - ((TimeCache.getInstance().get() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < LIGHT_COUNT; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= 0) {
        io.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void wave(
      double percent,
      Color c1,
      Color c2,
      double cycleLength,
      double duration,
      double waveExponent) {
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
        io.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private void bounce(
      double percent,
      Color c1,
      Color c2,
      double cycleLength,
      double duration,
      double waveExponent) {
    double x = (1 - ((TimeCache.getInstance().get() % duration) / duration) * 2.0 * Math.PI);
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < LIGHT_COUNT; i++) {
      if (i == 0) {
        System.out.println(x);
      }
      if (x > -2) {
        x += xDiffPerLed;
      } else {
        x -= xDiffPerLed;
      }
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
        io.setLED(i, new Color(red, green, blue));
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
        io.setLED(i, new Color(red, red, red));
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
    for (int i = StatusPage.ALL_SYSTEMS.length; i < LIGHT_COUNT; i++) {
      io.setLED(i, Color.kBlack);
    }
  }

  private void blink(Color color1, Color color2, double interval, double time) {
    if (time % (interval * 2.0) <= interval) {
      solid(1.0, color1);
    } else {
      solid(1.0, color2);
    }
  }

  private void alliance() {
    final var alliance = AllianceCache.getInstance().get();
    final boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    wave(100, isRed ? Color.kRed : Color.kBlue, Color.kBlack, 25.0, 2.0, 0.4);
  }

  private void matchTime() {
    final double autoSeconds = 15.0;
    final double teleopSeconds = 135.0;
    final double remaining = DriverStation.getMatchTime();
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
          io.setHSV(i, 0, 255, value);
        } else if (isLittleTimeLeft) {
          io.setHSV(i, 30, 255, value);
        } else {
          io.setHSV(i, 0, 0, value);
        }
      } else {
        io.setHSV(i, 0, 0, 0);
      }
    }
  }

  private void shooting() {
    final Pose2d pose = drivePoseSupplier.get();
    final ChassisSpeeds speeds = driveSpeedsSupplier.get();

    final ShotConfidence confidence = ShooterMath.calculateConfidence(pose, speeds);
    if (confidence.equals(ShotConfidence.LOW)) {
      solid(1.0, Color.kRed);
    } else if (confidence.equals(ShotConfidence.MEDIUM) && !isTargeted.get()) {
      solid(1.0, Color.kRed);
    } else if (confidence.equals(ShotConfidence.MEDIUM) && isTargeted.get()) {
      solid(1.0, Color.kOrange);
    } else if (confidence.equals(ShotConfidence.HIGH) && !isTargeted.get()) {
      solid(1.0, Color.kRed);
    } else if (confidence.equals(ShotConfidence.HIGH) && isTargeted.get()) {
      solid(1.0, Color.kGreen);
    } else if (timer.hasElapsed(0.5) && hasStartedTimer) {
      solid(1.0, Color.kOrange);
    }
  }

  private final Debouncer hasGamePieceBottomDebouncer = new Debouncer(0.10);
  private final Debouncer hasGamePieceTopDebouncer = new Debouncer(0.20);
  private final Timer startOfIntakeTimer = new Timer();
  private boolean hadGamePieceBottomBefore = false;

  private void delivery() {
    final double time = TimeCache.getInstance().get();
    final boolean inStow = this.inStow.get();
    final boolean inHandoff = this.inHandoff.get();
    final boolean hasGamePieceBottom =
        hasGamePieceBottomDebouncer.calculate(this.hasGamePieceBottom.get());
    final boolean hasGamePieceTop = hasGamePieceTopDebouncer.calculate(this.hasGamePieceTop.get());

    // Start beginning of intake timer
    if (!hadGamePieceBottomBefore && hasGamePieceBottom) {
      startOfIntakeTimer.restart();
    }
    SmartDashboard.putNumber("Lights/Start of Intake Timer", startOfIntakeTimer.get());
    hadGamePieceBottomBefore = hasGamePieceBottom && !hasGamePieceTop;

    if (hasGamePieceBottom && !hasGamePieceTop) {
      final double intakeBlinkInterval = 0.22;
      // Blink green when first intaking
      if (!startOfIntakeTimer.hasElapsed(intakeBlinkInterval * 5)) {
        SmartDashboard.putNumber("Lights/Intake Blink", startOfIntakeTimer.get());
        blink(Color.kGreen, Color.kBlack, intakeBlinkInterval, startOfIntakeTimer.get());
      } else {
        // Blink to notify a note is still in the intake
        blink(Color.kOrangeRed, Color.kBlack, intakeBlinkInterval, startOfIntakeTimer.get());
      }
      return;
    }

    // Blink white when in handoff
    if (inHandoff) {
      blink(Color.kWhite, Color.kBlack, 0.3, time);
      return;
    }

    // Solid orange when piece is in shooter
    if (hasGamePieceTop) {
      solid(1.0, Color.kOrangeRed);
      return;
    }

    // Purple when in stow
    if (inStow) {
      solid(1.0, Color.kPurple);
      return;
    }

    // Alliance otherwise
    alliance();
  }

  private void elevator() {
    final double percent = elevatorPercentageRaised.get();
    solid(1.0, Color.kBlack);
    solid(percent, Color.kOrangeRed);
  }

  private void pitTest() {
    if (pitTestFlashing) {
      blink(Color.kOrangeRed, Color.kBlack, 0.25, TimeCache.getInstance().get());
    }
    solid(pitTestStep / 10.0, Color.kGreen);
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
        timer.restart();
        hasStartedTimer = true;
      } else {
        hasStartedTimer = false;
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
}
