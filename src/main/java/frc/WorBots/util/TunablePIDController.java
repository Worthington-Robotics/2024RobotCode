package frc.WorBots.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * A PIDController that can be tuned by the dashboard.
 * It will hold a TunablePIDGains object and updates gains to the PID
 * every time it updates.
 */
public class TunablePIDController {
	private TunablePIDGains gains;
	/**
	 * The PIDController that is used. This can be safely accessed
	 * to run methods on it, however, you should use the update method
	 * of the tunable controller to ensure gains are updated
	 */
	public PIDController pid;

	/**
	 * Create a TunablePIDController and its gains with the specified NT location
	 * 
	 * @param table The NT table to put the gains on
	 * @param name The NT key to put the gains on
	 * @param kP The P constant
	 * @param kD The D constant
	 * @param kI The I constant
	 */
	public TunablePIDController(String table, String name, double kP, double kD, double kI) {
		this.gains = new TunablePIDGains(table, name, kP, kD, kI);
		this.pid = new PIDController(kP, kI, kD, this.gains.period);
	}

	/**
	 * Construct a TunablePIDController using existing gains.
	 * This is useful to be able to share a single static gains object
	 * between multiple controllers or instances
	 * 
	 * @param gains The gains to use
	 */
	public TunablePIDController(TunablePIDGains gains) {
		this.gains = gains;
		this.pid = new PIDController(gains.kP.getDefault(), gains.kI.getDefault(), gains.kD.getDefault(), gains.period);
	}

	/**
	 * Update gains to the latest tuned value
	 */
	public void update() {
		final double kP = this.gains.kP.get();
		final double kD = this.gains.kD.get();
		final double kI = this.gains.kI.get();
		pid.setPID(kP, kI, kD);
	}

	/**
	 * Change the gains that are used
	 * 
	 * @param gains The gains to set
	 */
	public void setGains(TunablePIDGains gains) {
		this.gains = gains;
		this.update();
	}

	/**
	 * Tunable PID gains that can be set by NetworkTables
	 */
	public static class TunablePIDGains {
		public TunableDouble kP;
		public TunableDouble kD;
		public TunableDouble kI;
		public double period;

		public TunablePIDGains(String table, String name) {
			this(table, name, 0.0, 0.0, 0.0);
		}

		public TunablePIDGains(String table, String name, double kP, double kD, double kI) {
			this(table, name, kP, kD, kI, 0.02);
		}

		public TunablePIDGains(String table, String name, double kP, double kD, double kI, double period) {
			this.kP = new TunableDouble(table, name, "kP", kP);
			this.kD = new TunableDouble(table, name, "kD", kD);
			this.kI = new TunableDouble(table, name, "kI", kI);
			this.period = period;
		}

		/**
		 * Set the gains
		 */
		public void setGains(double kP, double kD, double kI) {
			this.kP.set(kP);
			this.kD.set(kD);
			this.kI.set(kI);
		}
	}

	public static class TunableProfiledPIDController {
		private TunablePIDGains gains;
		private TunableTrapezoidConstraints constraints;
		public ProfiledPIDController pid;

		public TunableProfiledPIDController(String table, String name, double kP, double kD, double kI, double maxVelocity,
				double maxAcceleration) {
			this(new TunablePIDGains(table, name, kP, kD, kI),
					new TunableTrapezoidConstraints(table, name, maxVelocity, maxAcceleration));
		}

		public TunableProfiledPIDController(TunablePIDGains gains, TunableTrapezoidConstraints constraints) {
			this.gains = gains;
			this.constraints = constraints;
			this.pid = new ProfiledPIDController(gains.kP.getDefault(), gains.kD.getDefault(), gains.kI.getDefault(),
					new Constraints(this.constraints.maxVelocity.getDefault(), this.constraints.maxAcceleration.getDefault()),
					this.gains.period);
		}

		/**
		 * Update gains to the latest tuned value
		 */
		public void update() {
			final double kP = this.gains.kP.get();
			final double kD = this.gains.kD.get();
			final double kI = this.gains.kI.get();
			pid.setPID(kP, kI, kD);
			final Constraints constraints = this.constraints.makeConstraints();
			pid.setConstraints(constraints);
		}

		/**
		 * Change gains
		 */
		public void setGains(TunablePIDGains gains) {
			this.gains = gains;
			this.update();
		}

		/**
		 * Change constraints
		 */
		public void setConstraints(TunableTrapezoidConstraints constraints) {
			this.constraints = constraints;
			this.update();
		}
	}

	public static class TunableTrapezoidConstraints {
		public TunableDouble maxVelocity;
		public TunableDouble maxAcceleration;

		public TunableTrapezoidConstraints(String table, String name) {
			this(table, name, 0.0, 0.0);
		}

		public TunableTrapezoidConstraints(String table, String name, double maxVelocity, double maxAcceleration) {
			this.maxVelocity = new TunableDouble(table, name, "Max Velocity", maxVelocity);
			this.maxAcceleration = new TunableDouble(table, name, "Max Acceleration", maxAcceleration);
		}

		/**
		 * Set the constraints
		 */
		public void setConstraints(double maxVelocity, double maxAcceleration) {
			this.maxVelocity.set(maxVelocity);
			this.maxAcceleration.set(maxAcceleration);
		}

		public Constraints makeConstraints() {
			return new Constraints(maxVelocity.get(), maxAcceleration.get());
		}
	}
}
