package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.CORAL_INTAKE;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.*;

public class AlgaeBlaster extends GenericSubsystem {
    public Command setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState state) {
        return Commands.run(() -> ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotation2d().getRotations()), this)
                .alongWith(CORAL_INTAKE.rotateAlgaeBlasterEndEffector())
                .andThen(stopAlgaeBlasterArm());
    }

    public Command stopAlgaeBlasterArm() {
        return Commands.runOnce(ARM_BLASTER_MOTOR::stopMotor, this);
    }

    public boolean isArmAtTarget() {
        return ARM_BLASTER_MOTOR.isAtPositionSetpoint();
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        ARM_BLASTER_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return BLASTER_SYSID_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        ARM_BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor(ARM_BLASTER_MOTOR.getName())
                .voltage(Volts.of(ARM_BLASTER_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(ARM_BLASTER_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(ARM_BLASTER_MOTOR.getSystemVelocity()));
    }

    @Override
    public void periodic() {
        if (BLASTER_ARM_MECHANISM != null) {
            BLASTER_ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
            BLASTER_ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
        }
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_BLASTER_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(ARM_BLASTER_MOTOR.getClosedLoopTarget());
    }
}