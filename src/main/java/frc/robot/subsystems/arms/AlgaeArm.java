package frc.robot.subsystems.arms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.arms.AlgaeArmConstants.ALGAE_ARM_MOTOR;
import static frc.robot.subsystems.arms.AlgaeArmConstants.algaeArmMechanism;

public class AlgaeArm extends GenericSubsystem {
    /**
     * @param position in rotations
     */
    public Command setAlgaeArmPosition(double position) {
        return Commands.run(() -> ALGAE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position), this).andThen(stopAlgaeArm());
    }

    public Command stopAlgaeArm() {
        return Commands.runOnce(ALGAE_ARM_MOTOR::stopMotor, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(ALGAE_ARM_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(ALGAE_ARM_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        algaeArmMechanism.updateTargetAngle(getTargetPosition());
        algaeArmMechanism.updateCurrentAngle(getCurrentPosition());
    }
}
