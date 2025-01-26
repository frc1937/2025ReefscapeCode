package frc.robot.subsystems.algaearms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.algaearms.AlgaeIntakeConstants.*;

public class AlgaeIntake extends GenericSubsystem {
    public Command setAlgaeArmPosition(IntakeArmState state) {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.getRotation2d().getRotations()), this).andThen(stopAlgaeArm());
    }

    public Command setAlgaeIntakeVoltage(double voltage) {
        return Commands.run(() -> INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage), this);
    }

    public Command stopAlgaeArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    public Command stopAlgaeIntake() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor, this);
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget());
    }

    @Override
    public void periodic() {
        if (ARM_MECHANISM != null) {
            ARM_MECHANISM.updateTargetAngle(getTargetPosition());
            ARM_MECHANISM.updateCurrentAngle(getCurrentArmPosition());
        }
    }
}