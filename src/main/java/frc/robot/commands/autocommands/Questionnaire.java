package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeManipulationCommands;
import frc.robot.commands.CoralManipulationCommands;
import frc.robot.commands.PathfindingCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utilities.FieldConstants.Feeder;
import frc.robot.utilities.FieldConstants.ReefFace;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Questionnaire {
    private final LoggedDashboardChooser<ReefFace> QUESTION_1;
    private final LoggedDashboardChooser<Command>
            QUESTION_2,
            QUESTION_3,
            QUESTION_4,
            QUESTION_5;

    public Questionnaire() {
        QUESTION_1 = new LoggedDashboardChooser<>("Which Reef Face?");
        QUESTION_1.addOption("Face 0", ReefFace.FACE_0);
        QUESTION_1.addOption("Face 1", ReefFace.FACE_1);
        QUESTION_1.addOption("Face 2", ReefFace.FACE_2);
        QUESTION_1.addOption("Face 3", ReefFace.FACE_3);
        QUESTION_1.addOption("Face 4", ReefFace.FACE_4);
        QUESTION_1.addOption("Face 5", ReefFace.FACE_5);

        QUESTION_2 = new LoggedDashboardChooser<>("Which Branch?");
        QUESTION_2.addOption("Left Branch", PathfindingCommands.pathfindToLeftBranch(QUESTION_1.get()));
        QUESTION_2.addOption("Right Branch", PathfindingCommands.pathfindToRightBranch(QUESTION_1.get()));

        QUESTION_3 = new LoggedDashboardChooser<>("Should Remove Algae?");
        QUESTION_3.addOption("Yes", AlgaeManipulationCommands.blastAlgaeOffReef());
        QUESTION_3.addOption("No", Commands.none());

        QUESTION_4 = new LoggedDashboardChooser<>("Which Scoring Level?");
        QUESTION_4.addOption("L1", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L1));
        QUESTION_4.addOption("L2", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L2));
        QUESTION_4.addOption("L3", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L3));

        QUESTION_5 = new LoggedDashboardChooser<>("Which Feeder?");
        QUESTION_5.addOption("Top Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.TOP_FEEDER));
        QUESTION_5.addOption("Bottom Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.BOTTOM_FEEDER));
    }

    public Command getCommand() {
        return new SequentialCommandGroup(
                QUESTION_2.get(),
                QUESTION_3.get(),
                QUESTION_4.get(),
                QUESTION_5.get()
        ).repeatedly();
    }

    public String getSelected() {
        return "None";
    }
}
