package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.CoralIntakeConstants.*;

public class CoralIntake extends GenericSubsystem {
    public Command prepareGamePiece() {
        return Commands.run(() -> setVoltage(4), this).until(() -> BEAM_BREAK.get() == 1).andThen(stop());
    }

    public Command releaseGamePiece() {
        return Commands.run(() -> setVoltage(6), this).withTimeout(3.5);
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor);
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
        if (INTAKE_MECHANISM != null) {
            INTAKE_MECHANISM.updateCurrentSpeed(voltage);
        }
    }
}