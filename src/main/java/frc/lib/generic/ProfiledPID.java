package frc.lib.generic;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile. Users should
 * call reset() when they first start running the controller to avoid unwanted behavior.
 */
public class ProfiledPID {
    private PID m_controller;

    private double m_minimumInput;
    private double m_maximumInput;

    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     */
    public ProfiledPID(
            double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        this(Kp, Ki, Kd, 0, constraints);
    }

    public ProfiledPID(
            PIDConstants constants, double kS, TrapezoidProfile.Constraints constraints) {
        this(constants.kP, constants.kI, constants.kD, kS, constraints);
    }

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
     *
     * @param Kp The proportional coefficient.
     * @param Ki The integral coefficient.
     * @param Kd The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     * @param kS The minimum voltage value to apply
     * @throws IllegalArgumentException if kp &lt; 0
     * @throws IllegalArgumentException if ki &lt; 0
     * @throws IllegalArgumentException if kd &lt; 0
     * @throws IllegalArgumentException if period &lt;= 0
     */
    @SuppressWarnings("this-escape")
    public ProfiledPID(
            double Kp, double Ki, double Kd, double kS, TrapezoidProfile.Constraints constraints) {
        m_controller = new PID(Kp, Ki, Kd, kS);
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     * @param Ki The integral coefficient. Must be &gt;= 0.
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    public void setPID(double Kp, double Ki, double Kd) {
        m_controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return m_controller.getP();
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp The proportional coefficient. Must be &gt;= 0.
     */
    public void setP(double Kp) {
        m_controller.setP(Kp);
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return m_controller.getI();
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki The integral coefficient. Must be &gt;= 0.
     */
    public void setI(double Ki) {
        m_controller.setI(Ki);
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return m_controller.getD();
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd The differential coefficient. Must be &gt;= 0.
     */
    public void setD(double Kd) {
        m_controller.setD(Kd);
    }

    /**
     * Get the IZone range.
     *
     * @return Maximum magnitude of error to allow integral control.
     */
    public double getIZone() {
        return m_controller.getIZone();
    }

    /**
     * Sets the IZone range. When the absolute value of the position error is greater than IZone, the
     * total accumulated error will reset to zero, disabling integral gain until the absolute value of
     * the position error is less than IZone. This is used to prevent integral windup. Must be
     * non-negative. Passing a value of zero will effectively disable integral gain. Passing a value
     * of {@link Double#POSITIVE_INFINITY} disables IZone functionality.
     *
     * @param iZone Maximum magnitude of error to allow integral control.
     * @throws IllegalArgumentException if iZone &lt;= 0
     */
    public void setIZone(double iZone) {
        m_controller.setIZone(iZone);
    }

    /**
     * Returns the position tolerance of this controller.
     *
     * @return the position tolerance of the controller.
     */
    public double getPositionTolerance() {
        return m_controller.getPositionTolerance();
    }

    /**
     * Returns the velocity tolerance of this controller.
     *
     * @return the velocity tolerance of the controller.
     */
    public double getVelocityTolerance() {
        return m_controller.getVelocityTolerance();
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     *
     * @return The goal.
     */
    public TrapezoidProfile.State getGoal() {
        return m_goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        m_goal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    public void setGoal(double goal) {
        m_goal = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    public boolean atGoal() {
        return atSetpoint() && m_goal.equals(m_setpoint);
    }

    /**
     * Get the velocity and acceleration constraints for this controller.
     *
     * @return Velocity and acceleration constraints.
     */
    public TrapezoidProfile.Constraints getConstraints() {
        return m_constraints;
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        m_constraints = constraints;
        m_profile = new TrapezoidProfile(m_constraints);
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    public TrapezoidProfile.State getSetpoint() {
        return m_setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return True if the error is within the tolerance of the error.
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Enables continuous input.
     *
     * <p>Rather then using the max and min input range as constraints, it considers them to be the
     * same point and automatically calculates the shortest route to the setpoint.
     *
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        m_controller.enableContinuousInput(minimumInput, maximumInput);
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        m_controller.disableContinuousInput();
    }

    /**
     * Sets the minimum and maximum contributions of the integral term.
     *
     * <p>The internal integrator is clamped so that the integral term's contribution to the output
     * stays between minimumIntegral and maximumIntegral. This prevents integral windup.
     *
     * @param minimumIntegral The minimum contribution of the integral term.
     * @param maximumIntegral The maximum contribution of the integral term.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        m_controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The controller's next output.
     */
    public double calculate(double measurement) {
        if (m_controller.isContinuousInputEnabled()) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            double goalMinDistance =
                    MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance =
                    MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

            m_goal.position = goalMinDistance + measurement;
            m_setpoint.position = setpointMinDistance + measurement;
        }

        m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
        return m_controller.calculate(measurement, m_setpoint.position);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, TrapezoidProfile.State goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, double goal) {
        setGoal(goal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     * @return The controller's next output.
     */
    public double calculate(
            double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return calculate(measurement, goal);
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    public void reset(TrapezoidProfile.State measurement) {
        m_controller.reset();
        m_setpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is assumed to
     *     be zero.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }
}
