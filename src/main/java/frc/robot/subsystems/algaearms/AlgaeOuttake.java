package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.algaearms.AlgaeOuttakeConstants.ALGAE_OUTTAKE_MECHANISM;
import static frc.robot.subsystems.algaearms.AlgaeOuttakeConstants.OUTTAKE_MOTOR;

public class AlgaeOuttake extends GenericSubsystem {
    public Command setAlgaeRemoverPosition(AlgaeOuttakeConstants.ArmState state) {
        return Commands.run(() -> OUTTAKE_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotations()), this).andThen(stopAlgaeRemover());
    }

    public Command stopAlgaeRemover() {
        return Commands.runOnce(OUTTAKE_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(OUTTAKE_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(OUTTAKE_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        if (ALGAE_OUTTAKE_MECHANISM != null) {
            ALGAE_OUTTAKE_MECHANISM.updateTargetAngle(getTargetPosition());
            ALGAE_OUTTAKE_MECHANISM.updateCurrentAngle(getCurrentPosition());
        }
    }
}