package frc.robot.subsystems.algaeblaster;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.GlobalConstants.Mode.REAL;
import static frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.*;

public class AlgaeBlaster extends GenericSubsystem {
    public Command setAlgaeBlasterArmState(BlasterArmState state) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    BLASTER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotations());

                    if (CURRENT_MODE != REAL)
                        printPose(state.getRotations());
                },
                interrupt -> BLASTER_MOTOR.stopMotor(),
                BLASTER_MOTOR::isAtPositionSetpoint,
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


    private void printPose(double targetPosition) {
        final Pose3d current3dPose = new Pose3d(Pose3d.kZero.getTranslation(), new Rotation3d(0, getCurrentArmPosition().getRotations(), 0));

        Logger.recordOutput("Components/IntakeArmPose", current3dPose);

        if (BLASTER_ARM_MECHANISM != null) {
            BLASTER_ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
            BLASTER_ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
        }
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(BLASTER_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(BLASTER_MOTOR.getClosedLoopTarget());
    }
}