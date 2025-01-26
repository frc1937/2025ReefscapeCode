package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.algaearms.AlgaeOuttakeConstants.OUTTAKE_ARM_MECHANISM;
import static frc.robot.subsystems.algaearms.AlgaeOuttakeConstants.OUTTAKE_MOTOR;

public class AlgaeOuttake extends GenericSubsystem {
    public Command setAlgaeOuttakeArmState(AlgaeOuttakeConstants.OuttakeArmState state) {
        return Commands.run(() -> OUTTAKE_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotation2d().getRotations()), this).andThen(stopAlgaeOuttakeArm());
    }

    public Command stopAlgaeOuttakeArm() {
        return Commands.runOnce(OUTTAKE_MOTOR::stopMotor, this);
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(OUTTAKE_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(OUTTAKE_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        if (OUTTAKE_ARM_MECHANISM != null) {
            OUTTAKE_ARM_MECHANISM.updateTargetAngle(getTargetArmPosition());
            OUTTAKE_ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
        }
    }
}