package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.Arrays;

import static frc.robot.RobotContainer.ACCELEROMETER;

public class Optimizations {
    /**
     * Gets the skidding ratio from the latest module state, that can be used to determine how much the chassis is skidding
     * the skidding ratio is defined as the ratio between the maximum and minimum magnitude of the "translational" part of the speed of the modules
     *
     * @param swerveDriveKinematics the kinematics
     * @param swerveStatesMeasured  the swerve states measured from the modules
     * @return the skidding ratio, maximum/minimum, ranges from [1,INFINITY)
     */
    public static double getSkiddingRatio(final SwerveDriveKinematics swerveDriveKinematics, final SwerveModuleState[] swerveStatesMeasured) {
        final double angularVelocity = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
        final SwerveModuleState[] rotationalStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocity));

        final double[] translationalSpeeds = new double[swerveStatesMeasured.length];

        for (int i = 0; i < swerveStatesMeasured.length; i++) {
            final Translation2d measuredVelocity = convertToVelocityVector(swerveStatesMeasured[i]),
                    rotationalVelocity = convertToVelocityVector(rotationalStates[i]),
                    translationalVelocity = measuredVelocity.minus(rotationalVelocity);

            translationalSpeeds[i] = translationalVelocity.getNorm();
        }

        final double maxSpeed = Arrays.stream(translationalSpeeds).max().orElse(0);
        final double minSpeed = Arrays.stream(translationalSpeeds).min().orElse(Double.POSITIVE_INFINITY);

        return maxSpeed / minSpeed;
    }

    public static boolean isColliding() {
        final float xAccel = (float) ACCELEROMETER.getX();
        final float yAccel = (float) ACCELEROMETER.getY();
        return Math.hypot(xAccel, yAccel) * 9.8015 > 36;
    }

    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMPS the target velocity, in meters per second
     * @param targetSteerAngle  the target steer angle
     * @return the reduced target velocity in metres per second
     */
    public static double reduceSkew(double targetVelocityMPS, Rotation2d targetSteerAngle, Rotation2d currentAngle) {
        final double closedLoopError = targetSteerAngle.minus(currentAngle).getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));

        return targetVelocityMPS * cosineScalar;
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePositionRevolutions the position in revolutions
     * @param moduleAngle              the angle of the module
     * @return the distance without the coupling
     */
    public static double removeCouplingFromRevolutions(double drivePositionRevolutions, Rotation2d moduleAngle, double couplingRatio) {
        final double coupledAngle = moduleAngle.getRotations() * couplingRatio;
        return drivePositionRevolutions - coupledAngle;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    public static boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * Converts a {@link SwerveModuleState} to a {@link Translation2d} vector representing the velocity.
     * This method takes the speed and angle from the swerve module state and creates a 2D translation vector.
     * The speed is used as the magnitude of the vector, and the angle is used to determine the direction.
     *
     * @param state The {@link SwerveModuleState} to convert.
     * @return A {@link Translation2d} representing the velocity vector of the swerve module.
     */
    private static Translation2d convertToVelocityVector(final SwerveModuleState state) {
        return new Translation2d(state.speedMetersPerSecond, state.angle);
    }
}
