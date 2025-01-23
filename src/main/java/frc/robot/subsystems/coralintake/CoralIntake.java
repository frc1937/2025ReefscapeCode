package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.BEAM_BREAK_SENSOR;
import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_MOTOR;

public class CoralIntake extends GenericSubsystem {
    public Command prepareGamePiece() {
        return Commands.run(() -> setVoltage(4), this).until(() -> BEAM_BREAK_SENSOR.get() == 1).andThen(stop());
    }

    public Command releaseGamePiece() {
        return Commands.run(() -> setVoltage(6), this).withTimeout(3.5);
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("doesHoldGamePiece", BEAM_BREAK_SENSOR.get() == 1);
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}