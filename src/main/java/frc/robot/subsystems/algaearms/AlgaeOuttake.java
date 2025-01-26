package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.algaearms.AlgaeOuttakeConstants.ALGAE_OUTTAKE_MECHANISM;
import static frc.robot.subsystems.algaearms.AlgaeOuttakeConstants.OUTTAKE_MOTOR;

public class AlgaeOuttake extends GenericSubsystem {
    public Command setAlgaeOuttakeState(AlgaeOuttakeConstants.OuttakeArmState state) {
        return Commands.run(() -> OUTTAKE_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotation2d().getRotations()), this).andThen(stopAlgaeOuttake());
    }

    public Command stopAlgaeOuttake() {
        return Commands.runOnce(OUTTAKE_MOTOR::stopMotor, this);
    }

    private Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(OUTTAKE_MOTOR.getSystemPosition());
    }

     private Rotation2d getTargetPosition() {
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