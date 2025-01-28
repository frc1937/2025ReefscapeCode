package frc.robot.subsystems.algaeintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.algaeintake.AlgaeIntakeConstants.*;

public class AlgaeIntake extends GenericSubsystem {
    public Command setAlgaeIntakeArmState(IntakeArmState state) {
        return Commands.run(() -> INTAKE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotation2d().getRotations()), this)
                .andThen(stopAlgaeArm());
    }

    public Command setAlgaeIntakeVoltage(double voltage) {
        return Commands.run(() -> INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage), this);
    }

    public Command stopAlgaeArm() {
        return Commands.runOnce(INTAKE_ARM_MOTOR::stopMotor, this);
    }

    public Command stopAlgaeIntake() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor, this);
    }

    public boolean isArmAtTarget() {
        return INTAKE_ARM_MOTOR.isAtPositionSetpoint();
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


    @Override
    public void periodic() {
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