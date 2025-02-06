package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.GlobalConstants.Mode.REAL;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends GenericSubsystem {
    public Command setTargetHeight(Supplier<ElevatorHeight> levelSupplier) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    setMotorPosition(levelSupplier.get().getRotations());

                    if (CURRENT_MODE != REAL)
                        printPose();
                },
                interrupt -> stopMotors(),
                MASTER_MOTOR::isAtPositionSetpoint,
                this
        );
    }

    public Command setTargetHeight(ElevatorHeight level) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    setMotorPosition(level.getRotations());

                    if (CURRENT_MODE != REAL)
                        printPose();
                },
                interrupt -> stopMotors(),
                MASTER_MOTOR::isAtPositionSetpoint,
                this
        );
    }

    public boolean isAtTarget() {
        return MASTER_MOTOR.isAtPositionSetpoint();
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        MASTER_MOTOR.setIdleMode(idleMode);
        SLAVE_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public void periodic() {
        if (BOTTOM_BEAM_BREAK.get() == 1)
            MASTER_MOTOR.setMotorEncoderPosition(0);

        if (TOP_BEAM_BREAK.get() == 1)
            MASTER_MOTOR.setMotorEncoderPosition(ELEVATOR_MAX_EXTENSION_METERS);
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

    private void printPose() {
        final double currentElevatorPosition = Conversions.rotationsToMetres(MASTER_MOTOR.getSystemPosition(), WHEEL_DIAMETER);
        final double targetElevatorPosition = Conversions.rotationsToMetres(MASTER_MOTOR.getClosedLoopTarget(), WHEEL_DIAMETER);
        final Pose3d current3dPose = new Pose3d(0, 0, currentElevatorPosition, new Rotation3d(0, 0, 0));

        Logger.recordOutput("Components/ElevatorPose", current3dPose);

        ELEVATOR_MECHANISM.updateCurrentPosition(currentElevatorPosition);
        ELEVATOR_MECHANISM.updateTargetPosition(targetElevatorPosition);
    }

    private void setMotorPosition(double targetPosition) {
        MASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }

    private void stopMotors() {
        MASTER_MOTOR.stopMotor();
    }
}
