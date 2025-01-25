package frc.robot.subsystems.arms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.arms.AlgaeRemoverConstants.ALGAE_REMOVER_MOTOR;
import static frc.robot.subsystems.arms.AlgaeRemoverConstants.algaeRemoverMechanism;

public class AlgaeRemover extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setAlgaeRemoverPosition(double position) {
        return Commands.run(() -> ALGAE_REMOVER_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this).andThen(stopAlgaeRemover());
    }

    public Command stopAlgaeRemover() {
        return Commands.runOnce(ALGAE_REMOVER_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(ALGAE_REMOVER_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(ALGAE_REMOVER_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        algaeRemoverMechanism.updateTargetAngle(getTargetPosition());
        algaeRemoverMechanism.updateCurrentAngle(getCurrentPosition());
    }
}
