package frc.lib.generic.characterization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;

public class StaticFrictionCharacterization extends Command {
    private final Motor motor;

    private final boolean shouldInvertVoltage;

    private int movedCounter;
    private double voltage;

    public StaticFrictionCharacterization(GenericSubsystem requirement, Motor motor, boolean shouldInvertVoltage) {
        this.motor = motor;
        this.shouldInvertVoltage = shouldInvertVoltage;

        movedCounter = 0;

        addRequirements(requirement);
    }

    @Override
    public void initialize() {
        motor.stopMotor();

        movedCounter = 0;
        voltage = 0;
    }

    @Override
    public void execute() {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, shouldInvertVoltage ? -voltage : voltage);

        if (Math.abs(motor.getSystemVelocity()) > 0.01) {
            System.out.print(
                    "\n<~~~~~~~~~~~~~~>" +
                    "\nMECHANISM " + motor.getName() + " MOVED AT " + voltage + " with velocity of " + motor.getSystemVelocity() +
                    "\n<~~~~~~~~~~~~~~>");

            movedCounter++;
        } else {
             voltage += 0.001;

            System.out.println("MECHANISM " + motor.getName() + "IS STILL NOT MOVING AT SPEED: " + voltage + " SPEED: " + motor.getSystemVelocity() + " AND VOLTAGE " + motor.getVoltage());
        }
    }

    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return movedCounter > 4;
    }
}
