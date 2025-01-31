package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.algaeintake.AlgaeIntakeConstants;

import static frc.robot.RobotContainer.ALGAE_BLASTER;
import static frc.robot.RobotContainer.ALGAE_INTAKE;

public class AlgaeManipulationCommands {
    public static Command intakeAlgae() {
        return ALGAE_INTAKE.setAlgaeIntakeState(AlgaeIntakeConstants.IntakeState.EXTENDED)
                .raceWith(new WaitCommand(2))
                .andThen(ALGAE_INTAKE.setAlgaeIntakeState(AlgaeIntakeConstants.IntakeState.RETRACTED));
    }

    public static Command releaseAlgae() {
        return ALGAE_INTAKE.setRollersVoltage(6)
                .raceWith(new WaitCommand(1.2));
    }

    public static Command blastAlgaeOffReef() {
        return ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT)
                .until(ALGAE_BLASTER::isArmAtTarget)
                .andThen(new WaitCommand(0.4))
                .andThen(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN));
    }
}
