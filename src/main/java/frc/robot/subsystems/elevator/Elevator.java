package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends GenericSubsystem {
    /**
     * This commands NEVER ENDS as the elevator needs to maintain its position.
     *
     * @param levelSupplier the target height of the elevator as a supplier
     * @return the command to set the target height
     */
    public Command setTargetHeight(Supplier<ElevatorHeight> levelSupplier) {
        return new FunctionalCommand(
                () -> {},
                () -> setMotorPosition(levelSupplier.get().getRotations()),
                interrupt -> stopMotors(),
                () -> false,
                this
        );
    }

    /**
     * This commands NEVER ENDS as the elevator needs to maintain its position.
     *
     * @param level the target height of the elevator
     * @return the command to set the target height
     */
    public Command setTargetHeight(ElevatorHeight level) {
        return new FunctionalCommand(
                () -> {},
                () -> setMotorPosition(level.getRotations()),
                interrupt -> stopMotors(),
                () -> isAtTargetHeight(level),
                this
        );
    }

    public boolean isAtTargetHeight(ElevatorHeight level) {
        return Math.abs(MASTER_MOTOR.getSystemPosition() - level.getRotations()) < 0.1;
    }

    public boolean isAtTargetPosition() {
        return MASTER_MOTOR.isAtPositionSetpoint();
    }

    public double getCurrentHeight() {
        return Conversions.rotationsToMetres(MASTER_MOTOR.getSystemPosition(), WHEEL_DIAMETER);
    }

    public Command runCurrentZeroing() {
        final int[] count = {0};
        final Timer timer = new Timer();

        return new FunctionalCommand(
                () -> {
                    timer.restart();
                    count[0] = 0;
                },
                () -> MASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 1),
                (interrupt) -> {
                    MASTER_MOTOR.stopMotor();
                    MASTER_MOTOR.setMotorEncoderPosition(ELEVATOR_MAX_EXTENSION_ROTATIONS );
                },
                () -> {
                    if (MASTER_MOTOR.getCurrent() > 31.0) count[0]++;
                    else count[0] = 0;

                    return count[0] > 4 && timer.hasElapsed(0.1);
                },
                this
        );
    }

    public Command runElevatorDownwards() {
        return new FunctionalCommand(
                () -> {},
                () -> MASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, -1.5),
                (interrupt) -> stop(),
                () -> false,
                this
        );
    }

    public Command runElevatorUpwards() {
        return new FunctionalCommand(
                () -> {},
                () -> MASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 1.5),
                (interrupt) -> stop(),
                () -> false,
                this
        );
    }

    public void stop() {
        MASTER_MOTOR.stopMotor();
        SLAVE_MOTOR.stopMotor();
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        MASTER_MOTOR.setIdleMode(idleMode);
        SLAVE_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ELEVATOR_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        MASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("ELEVATOR_MOTOR")
                .voltage(Volts.of(MASTER_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(MASTER_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(MASTER_MOTOR.getSystemVelocity()));
    }

    public void printPose() {
        if (ELEVATOR_MECHANISM != null) {
            final double currentElevatorPosition = Conversions.rotationsToMetres(MASTER_MOTOR.getSystemPosition(), WHEEL_DIAMETER);
            final double targetElevatorPosition = Conversions.rotationsToMetres(MASTER_MOTOR.getClosedLoopTarget(), WHEEL_DIAMETER);
            final Pose3d current3dPose = new Pose3d(0, 0, currentElevatorPosition / 2, new Rotation3d(0, 0, 0));

            Logger.recordOutput("Components/ElevatorPose", current3dPose);

            ELEVATOR_MECHANISM.updateCurrentPosition(currentElevatorPosition);
            ELEVATOR_MECHANISM.updateTargetPosition(targetElevatorPosition);
        }
    }

    private void setMotorPosition(double targetPosition) {
        MASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }

    private void stopMotors() {
        MASTER_MOTOR.stopMotor();
        SLAVE_MOTOR.stopMotor();
    }
}