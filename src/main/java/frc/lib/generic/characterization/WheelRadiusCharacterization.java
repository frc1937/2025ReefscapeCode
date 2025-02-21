package frc.lib.generic.characterization;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.robot.subsystems.swerve.SwerveCommands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Arrays;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class WheelRadiusCharacterization extends Command {
    private static final LoggedNetworkNumber CHARACTERIZATION_SPEED = new LoggedNetworkNumber("/SmartDashboard/RadiusCharacterization/SpeedRadiansPerSecond", 1.0);
    private static final LoggedNetworkNumber ROTATION_RATE_LIMIT = new LoggedNetworkNumber("/SmartDashboard/RadiusCharacterization/RotationRateLimit", 1.0);
    private static final LoggedNetworkBoolean SHOULD_MOVE_CLOCKWISE = new LoggedNetworkBoolean("/SmartDashboard/RadiusCharacterization/ShouldMoveClockwise", false);

    private final double[] wheelDistancesFromCenterMeters;

    private final Supplier<double[]> wheelPositionsRadiansSupplier;
    private final DoubleSupplier gyroYawRadiansSupplier;
    private final DoubleConsumer runWheelRadiusCharacterization;

    private SlewRateLimiter rotationSlewRateLimiter;

    private double gyroStartingYawRadians;
    private double accumulatedYawRadians;

    private double[] startingWheelPositions;
    private double currentDriveWheelRadius;

    public WheelRadiusCharacterization(GenericSubsystem requirement, Translation2d[] wheelDistancesFromCenterMeters, Supplier<double[]> wheelPositionsRadiansSupplier, DoubleSupplier gyroYawRadiansSupplier, DoubleConsumer runWheelRadiusCharacterization) {
        this.wheelDistancesFromCenterMeters = Arrays.stream(wheelDistancesFromCenterMeters).mapToDouble(Translation2d::getNorm).toArray();
        this.wheelPositionsRadiansSupplier = wheelPositionsRadiansSupplier;
        this.gyroYawRadiansSupplier = gyroYawRadiansSupplier;
        this.runWheelRadiusCharacterization = runWheelRadiusCharacterization;

        addRequirements(requirement);
    }

    public WheelRadiusCharacterization(GenericSubsystem requirement, double[] wheelDistancesFromCenterMeters, Supplier<double[]> wheelPositionsRadiansSupplier, DoubleSupplier gyroYawRadiansSupplier, DoubleConsumer runWheelRadiusCharacterization) {
        this.wheelDistancesFromCenterMeters = wheelDistancesFromCenterMeters;
        this.wheelPositionsRadiansSupplier = wheelPositionsRadiansSupplier;
        this.gyroYawRadiansSupplier = gyroYawRadiansSupplier;
        this.runWheelRadiusCharacterization = runWheelRadiusCharacterization;

        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        SwerveCommands.driveOpenLoop(() -> 0, () -> 0, () -> 0.1, () -> true);

        gyroStartingYawRadians = gyroYawRadiansSupplier.getAsDouble();
        startingWheelPositions = wheelPositionsRadiansSupplier.get();
        accumulatedYawRadians = 0.0;

        rotationSlewRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT.get());
        rotationSlewRateLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        runWheelRadiusCharacterization.accept(rotationSlewRateLimiter.calculate(getRotationDirection() * CHARACTERIZATION_SPEED.get()));
        accumulatedYawRadians = Math.abs(gyroStartingYawRadians - gyroYawRadiansSupplier.getAsDouble());
        calculateDriveWheelRadius();

        Logger.recordOutput("RadiusCharacterization/AccumulatedGyroYawRadians", accumulatedYawRadians);
        Logger.recordOutput("RadiusCharacterization/DriveWheelRadius", currentDriveWheelRadius);
    }

    @Override
    public void end(boolean interrupted) {
        if (accumulatedYawRadians <= 6.283185307179586) {
            System.out.println("Not enough data for characterization. Rotate at least a full rotation nigga.");
        } else {
            System.out.println("Drive Wheel Radius: " + currentDriveWheelRadius + " meters");
        }
    }

    private void calculateDriveWheelRadius() {
        currentDriveWheelRadius = 0.0;
        double[] wheelPositionsRadians = wheelPositionsRadiansSupplier.get();

        for(int i = 0; i < 4; ++i) {
            double accumulatedWheelRadians = Math.abs(wheelPositionsRadians[i] - startingWheelPositions[i]);
            currentDriveWheelRadius += accumulatedYawRadians * wheelDistancesFromCenterMeters[i] / accumulatedWheelRadians;
            Logger.recordOutput("RadiusCharacterization/AccumulatedWheelRadians" + i, accumulatedWheelRadians);
        }

        currentDriveWheelRadius = currentDriveWheelRadius / 4.0 ;
    }

    private int getRotationDirection() {
        return SHOULD_MOVE_CLOCKWISE.get() ? -1 : 1;
    }
}
