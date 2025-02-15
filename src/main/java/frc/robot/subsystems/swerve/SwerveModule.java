package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderInputs;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.lib.math.Optimizations;

import static edu.wpi.first.units.Units.*;
import static frc.lib.math.Conversions.rotationsToMetres;
import static frc.robot.GlobalConstants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.WHEEL_DIAMETER;
import static frc.robot.utilities.PathPlannerConstants.ROBOT_CONFIG;

public class SwerveModule {
    private final Motor steerMotor, driveMotor;
    private final Encoder steerEncoder;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(Motor driveMotor, Motor steerMotor, Encoder steerEncoder) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.steerEncoder = steerEncoder;
    }

    /**
     * SETS RAW VOLTAGE TO THE DRIVE MOTOR! UNSAFE! Only use FOR CHARACTERIZATION!
     * @param voltage The voltage the drive motor receives
     */
    protected void runDriveMotorForCharacterization(double voltage) {
        driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    protected double getDriveWheelPositionRadians() {
        return 2 * Math.PI * getDriveMotorInputs().threadSystemPosition[
                getDriveMotorInputs().threadSystemPosition.length - 1];
    }

    protected void logForSysId(SysIdRoutineLog log) {
        log.motor("DRIVE_MOTOR_SWERVE" + driveMotor.getDeviceID())
                .voltage(Volts.of(driveMotor.getVoltage()))
                .angularPosition(Rotations.of(driveMotor.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(driveMotor.getSystemVelocity()));
    }

    protected void setTargetState(SwerveModuleState state, boolean shouldUseClosedLoop) {
        this.targetState = Optimizations.optimize(state, getCurrentAngle());

        targetState.speedMetersPerSecond = Optimizations.reduceSkew(targetState.speedMetersPerSecond, targetState.angle, getCurrentAngle());

        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond, shouldUseClosedLoop);
    }

    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    protected SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        if (getDriveMotorInputs().threadSystemPosition.length != getSteerEncoderInputs().threadPosition.length) {
            return null;
        }

        return new SwerveModulePosition(
                getDriveMetersTraveled(getDriveMotorInputs().threadSystemPosition)[odometryUpdateIndex],
                Rotation2d.fromRotations(getSteerEncoderInputs().threadPosition[odometryUpdateIndex])
        );
    }

    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    protected void setTargetVelocity(double velocityMetresPerSecond, boolean shouldUseClosedLoop) {
        if (!isTemperatureOkay()) System.out.println("SWERVE MODULE " + driveMotor.getDeviceID() + " is TOO HOT!" );

        if (shouldUseClosedLoop) {
            final double targetVelocityRPSClosedLoop = Conversions.mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER);
            driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPSClosedLoop);
        } else {
            final double targetPowerOpenLoop = VOLTAGE_COMPENSATION_SATURATION *
                    (velocityMetresPerSecond / ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS);

            driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, targetPowerOpenLoop);
        }
    }

    protected void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(Conversions.rpsToMps(driveMotor.getSystemVelocity(), WHEEL_DIAMETER), getCurrentAngle());
    }

    protected SwerveModuleState getTargetState() {
        return targetState;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerEncoder.getEncoderPosition());
    }

    private EncoderInputs getSteerEncoderInputs() {
        return steerEncoder.getInputs();
    }

    private MotorInputs getDriveMotorInputs() {
        return driveMotor.getInputs();
    }

    private double[] getDriveMetersTraveled(double[] rotationsPositions) {
        final double[] metersTraveled = new double[rotationsPositions.length];

        for (int i = 0; i < rotationsPositions.length; i++) {
            metersTraveled[i] = rotationsToMetres(rotationsPositions[i], WHEEL_DIAMETER);
        }

        return metersTraveled;
    }

    private boolean isTemperatureOkay() {
        return driveMotor.getTemperature() < 80;
    }
}
