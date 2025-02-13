package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.AutoLogOutput;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_BEAM_BREAK;
import static frc.robot.subsystems.coralintake.CoralIntakeConstants.INTAKE_MOTOR;

public class CoralIntake extends GenericSubsystem {
    public Command prepareGamePiece() {
        return Commands.run(() -> setVoltage(4), this).until(this::hasCoral).andThen(stop());
    }

    public Command releaseGamePiece() {
        return Commands.run(() -> setVoltage(6), this).withTimeout(3.5);
    }

    public Command rotateAlgaeBlasterEndEffector() {
        return Commands.run(() -> setVoltage(-2)).withTimeout(2);
    }

    public Command stop() {
        return Commands.runOnce(INTAKE_MOTOR::stopMotor);
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        INTAKE_MOTOR.setIdleMode(idleMode);
    }

    @AutoLogOutput(key = "hasCoral")
    public boolean hasCoral() {
        return INTAKE_BEAM_BREAK.get() == 1;
    }

    private void setVoltage(double voltage) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }
}