package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_MOTOR;

public class Climb extends GenericSubsystem {
    public Command setClimbState(ClimbConstants.ClimbState state) {
        return new FunctionalCommand(
                () -> {},
                () -> CLIMB_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, state.positionRotations),
                (interrupt) -> CLIMB_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    public Command runVoltage(double voltage) {
        return new FunctionalCommand(
                () -> {},
                () -> CLIMB_MOTOR.setOutput(MotorProperties.ControlMode.VELOCITY, voltage),
                (interrupt) -> CLIMB_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }
}
