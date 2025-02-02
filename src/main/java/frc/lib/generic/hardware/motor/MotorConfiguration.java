package frc.lib.generic.hardware.motor;

import frc.lib.generic.simulation.SimulationProperties;

/**
 * Represents a generic motor configuration with default values.
 *
 * <p>This class holds various motor settings, including inversion, control modes,
 * simulation properties, ramp rates, soft limits, motion profile constraints,
 * and closed-loop control settings.</p>
 *
 * <p>It is designed to be used with motor controllers in robotics applications,
 * providing a structured way to define and modify motor behavior.</p>
 */
public class MotorConfiguration {

    /** Determines whether the motor output is inverted. Default: {@code false}. */
    public boolean inverted = false;

    /** The motor's idle mode, determining its behavior when no input is applied. Default: {@code BRAKE}. */
    public MotorProperties.IdleMode idleMode = MotorProperties.IdleMode.BRAKE;

    /** PID slot configuration for closed-loop control. */
    public MotorProperties.Slot slot = new MotorProperties.Slot(0, 0, 0, 0, 0, 0, 0, null);

    /** Simulation-specific motor properties. */
    public SimulationProperties.Slot simulationProperties = new SimulationProperties.Slot(null, null, 0, 0);

    /** Separate PID slot for simulation use. */
    public MotorProperties.Slot simulationSlot = new MotorProperties.Slot(0, 0, 0, 0, 0, 0, 0, null);

    /**
     * Time to ramp from 0% to 100% output in open-loop mode.
     *
     * <ul>
     *   <li><b>Range:</b> 0 to 1</li>
     *   <li><b>Default:</b> 0</li>
     *   <li><b>Units:</b> seconds</li>
     * </ul>
     */
    public double dutyCycleOpenLoopRampPeriod = 0;

    /**
     * Time to ramp from 0% to 100% output in closed-loop mode.
     *
     * <ul>
     *   <li><b>Range:</b> 0 to 1</li>
     *   <li><b>Default:</b> 0</li>
     *   <li><b>Units:</b> seconds</li>
     * </ul>
     */
    public double dutyCycleClosedLoopRampPeriod = 0;

    /**
     * Enables continuous wrap for closed-loop position control.
     * <p>Used for mechanisms like swerve azimuth to handle continuous rotations.</p>
     * <p><b>Default:</b> {@code false}</p>
     */
    public boolean closedLoopContinuousWrap = false;

    /** Current limits for supply and stator. Default: {@code -1} (disabled). */
    public double supplyCurrentLimit = -1, statorCurrentLimit = -1;

    /**
     * Gear ratio defining the conversion between system rotations and motor rotations.
     *
     * <ul>
     *   <li><b>Minimum:</b> > 0</li>
     *   <li><b>Default:</b> 1</li>
     *   <li><b>Units:</b> SYSTEM/MOTOR (ratio)</li>
     * </ul>
     */
    public double gearRatio = 1;

    /**
     * Maximum velocity for motion profiling.
     *
     * <ul>
     *   <li><b>Range:</b> 0 to 9999</li>
     *   <li><b>Default:</b> 0</li>
     *   <li><b>Units:</b> rotations per second (rps)</li>
     * </ul>
     */
    public double profileMaxVelocity = 0;

    /**
     * Target acceleration for motion profiling.
     *
     * <ul>
     *   <li><b>Range:</b> 0 to 9999</li>
     *   <li><b>Default:</b> 0</li>
     *   <li><b>Units:</b> rotations per second²</li>
     * </ul>
     */
    public double profileMaxAcceleration = 0;

    /**
     * Maximum jerk for motion profiling.
     *
     * <ul>
     *   <li><b>Range:</b> 0 to 9999</li>
     *   <li><b>Default:</b> 0</li>
     *   <li><b>Units:</b> rotations per second²</li>
     * </ul>
     */
    public double profileMaxJerk = 0;

    /**
     * Tolerance for closed-loop control, used to determine if the target is reached.
     *
     * <p>If not set, {@link Motor#isAtPositionSetpoint()} and {@link Motor#isAtVelocitySetpoint()} will throw an exception.</p>
     *
     * <ul>
     *   <li><b>Minimum:</b> 0</li>
     *   <li><b>Default:</b> 0</li>
     *   <li><b>Units:</b> rotations</li>
     * </ul>
     *
     * <p>Ignored if set to 0.</p>
     */
    public double closedLoopTolerance = 0;

    /** Forward and reverse soft limits for motor movement. Default: {@code 0}. */
    public Double forwardSoftLimit = null, reverseSoftLimit = null;
}
