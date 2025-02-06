package frc.robot.subsystems.algaeintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.algaeintake.AlgaeIntakeConstants.*;

public class AlgaeIntake extends GenericSubsystem {
    public Command setAlgaeIntakeState(IntakeState state) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getTargetArmPositionRotations());
                    INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, state.getRollerVoltage());
                },
                interrupt -> {
                    INTAKE_MOTOR.stopMotor();
                    INTAKE_ARM_MOTOR.stopMotor();
                },
                () -> false,
                this
        );
    }

    public Command setRollersVoltage(double voltage) {
        return Commands.run(() -> INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage), this);
    }

    public boolean isArmAtTarget() {
        return INTAKE_ARM_MOTOR.isAtPositionSetpoint();
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        INTAKE_MOTOR.setIdleMode(idleMode);
        INTAKE_ARM_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return INTAKE_ARM_SYSID_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor(INTAKE_ARM_MOTOR.getName())
                .voltage(Volts.of(INTAKE_ARM_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(INTAKE_ARM_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(INTAKE_ARM_MOTOR.getSystemVelocity()));
    }

    private void printPose() {
        final Pose3d current3dPose = new Pose3d(Pose3d.kZero.getTranslation(), new Rotation3d(0, getCurrentArmPosition().getRotations(), 0));

        Logger.recordOutput("Components/IntakeArmPose", current3dPose);

        if (INTAKE_ARM_MECHANISM != null) {
            INTAKE_ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
            INTAKE_ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
        }
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(INTAKE_ARM_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(INTAKE_ARM_MOTOR.getClosedLoopTarget());
    }
}