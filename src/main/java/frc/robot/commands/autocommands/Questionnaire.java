package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathfindingCommands;
import frc.robot.subsystems.algaeblaster.AlgaeBlasterConstants.BlasterArmState;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight;
import frc.robot.utilities.FieldConstants.ReefFace;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.RobotContainer.*;


public class Questionnaire {
    private final LoggedDashboardChooser<Command>
            QUESTION_1,
            QUESTION_2,
            QUESTION_3,
            QUESTION_4,
            QUESTION_5;

    public Questionnaire() {
        QUESTION_1 = new LoggedDashboardChooser<>("Which Reef Face?");
        QUESTION_1.addOption("Face 0", PathfindingCommands.pathfindToFace(ReefFace.FACE_0));
        QUESTION_1.addOption("Face 1", PathfindingCommands.pathfindToFace(ReefFace.FACE_1));
        QUESTION_1.addOption("Face 2", PathfindingCommands.pathfindToFace(ReefFace.FACE_2));
        QUESTION_1.addOption("Face 3", PathfindingCommands.pathfindToFace(ReefFace.FACE_3));
        QUESTION_1.addOption("Face 4", PathfindingCommands.pathfindToFace(ReefFace.FACE_4));
        QUESTION_1.addOption("Face 5", PathfindingCommands.pathfindToFace(ReefFace.FACE_5));

        QUESTION_2 = new LoggedDashboardChooser<>("Which Branch?");
        QUESTION_2.addOption("Left Branch", PathfindingCommands.pathfindToLeftBranch());
        QUESTION_2.addOption("Right Branch", PathfindingCommands.pathfindToRightBranch());

        QUESTION_3 = new LoggedDashboardChooser<>("Should Remove Algae?");
        QUESTION_3.addOption("Yes", ALGAE_BLASTER.setAlgaeBlasterArmState(BlasterArmState.HORIZONTAL_OUT));
        QUESTION_3.addOption("No", ALGAE_BLASTER.setAlgaeBlasterArmState(BlasterArmState.HORIZONTAL_IN));

        QUESTION_4 = new LoggedDashboardChooser<>("Which Scoring Level?");
        QUESTION_4.addOption("L1",
                ELEVATOR.setTargetHeight(ElevatorHeight.L1).alongWith(
                        CORAL_INTAKE.prepareGamePiece()).andThen(
                        CORAL_INTAKE.releaseGamePiece()));
        QUESTION_4.addOption("L2",
                ELEVATOR.setTargetHeight(ElevatorHeight.L2).alongWith(
                        CORAL_INTAKE.prepareGamePiece()).andThen(
                        CORAL_INTAKE.releaseGamePiece()));
        QUESTION_4.addOption("L3",
                ELEVATOR.setTargetHeight(ElevatorHeight.L3).alongWith(
                        CORAL_INTAKE.prepareGamePiece()).andThen(
                        CORAL_INTAKE.releaseGamePiece()));

        QUESTION_5 = new LoggedDashboardChooser<>("Which Feeder?");
        QUESTION_5.addOption("Top Feeder", PathfindingCommands.pathfindToFeeder(true));
        QUESTION_5.addOption("Bottom Feeder", PathfindingCommands.pathfindToFeeder(false));
    }

    public Command getCommand() {
        return new SequentialCommandGroup(
                QUESTION_1.get(),
                QUESTION_2.get(),
                QUESTION_3.get(),
                QUESTION_4.get(),
                QUESTION_5.get()
        );
    }

    public String getSelected() {
        return "None";
    }
}
