package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.ELEVATOR;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.*;

public class AlgaeBlaster extends GenericSubsystem {
    public Command setAlgaeBlasterState(BlasterArmState state) {
        return new FunctionalCommand(
                () -> {
                },
                () -> BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotations()),
                interrupt -> BLASTER_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command stopAlgaeBlasterArm() {
        return Commands.runOnce(BLASTER_MOTOR::stopMotor, this);
    }

    public boolean isArmAtTarget() {
        return BLASTER_MOTOR.isAtPositionSetpoint();
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        BLASTER_MOTOR.setIdleMode(idleMode);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return BLASTER_SYSID_CONFIG;
    }

    @Override
    public void sysIdDrive(double voltage) {
        BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor(BLASTER_MOTOR.getName())
                .voltage(Volts.of(BLASTER_MOTOR.getVoltage()))
                .angularPosition(Rotations.of(BLASTER_MOTOR.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(BLASTER_MOTOR.getSystemVelocity()));
    }

    public void printPose() {
        if (BLASTER_ARM_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(new Translation3d(0.279, 0.31, ELEVATOR.getCurrentHeight() / 2 + 0.81), new Rotation3d(0, getCurrentArmPosition().getRadians() - Math.PI / 2, Math.PI / 2));

            Logger.recordOutput("Components/BlasterArmPose", current3dPose);

            BLASTER_ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
            BLASTER_ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
        }
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(BLASTER_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(BLASTER_MOTOR.getClosedLoopTarget());
    }
}