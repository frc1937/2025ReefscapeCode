package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utilities.FieldConstants;

import static frc.robot.RobotContainer.*;

public class AlgaeManipulationCommands {
    /**
     * Sets the elevator to the height of the algae, then removes the algae.
     *
     * @return the command
     */
    public static Command blastAlgaeOffReefWithElevator(FieldConstants.ReefFace face) {
        return ELEVATOR.setTargetHeight(getAlgaeHeightFromFace(face))
                .andThen(blastAlgaeOffReef())
                .raceWith((ELEVATOR.maintainPosition()));
    }

    public static Command blastAlgaeOffReef() {
        return CORAL_INTAKE.rotateAlgaeBlasterEndEffector()
                .alongWith(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_OUT))
                .withTimeout(1.5)
                .andThen(ALGAE_BLASTER.setAlgaeBlasterArmState(AlgaeBlasterConstants.BlasterArmState.HORIZONTAL_IN));
    }

    public static ElevatorConstants.ElevatorHeight getAlgaeHeightFromFace(FieldConstants.ReefFace face) {
        if (face.ordinal() % 2 == 0)
            return ElevatorConstants.ElevatorHeight.REMOVE_ALGAE_FROM_L3;
        return ElevatorConstants.ElevatorHeight.L2;
    }
}
