package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlgaeManipulationCommands;
import frc.robot.commands.CoralManipulationCommands;
import frc.robot.commands.PathfindingCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utilities.FieldConstants.Feeder;
import frc.robot.utilities.FieldConstants.ReefFace;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Questionnaire {
    private final LoggedDashboardChooser<ReefFace> CYCLE_1_QUESTION_1, CYCLE_2_QUESTION_1, CYCLE_3_QUESTION_1;
    private final LoggedDashboardChooser<String> CYCLE_1_QUESTION_2, CYCLE_2_QUESTION_2, CYCLE_3_QUESTION_2;
    private final LoggedDashboardChooser<Command>
            CYCLE_1_QUESTION_3, CYCLE_1_QUESTION_4, CYCLE_1_QUESTION_5,
            CYCLE_2_QUESTION_3, CYCLE_2_QUESTION_4, CYCLE_2_QUESTION_5,
            CYCLE_3_QUESTION_3, CYCLE_3_QUESTION_4, CYCLE_3_QUESTION_5;

    public Questionnaire() {
        CYCLE_1_QUESTION_1 = createReefFaceQuestion("Cycle 1");
        CYCLE_1_QUESTION_2 = createBranchQuestion("Cycle 1");
        CYCLE_1_QUESTION_3 = createAlgaeQuestion("Cycle 1");
        CYCLE_1_QUESTION_4 = createScoringQuestion("Cycle 1");
        CYCLE_1_QUESTION_5 = createFeederQuestion("Cycle 1");

        CYCLE_2_QUESTION_1 = createReefFaceQuestion("Cycle 2");
        CYCLE_2_QUESTION_2 = createBranchQuestion("Cycle 2");
        CYCLE_2_QUESTION_3 = createAlgaeQuestion("Cycle 2");
        CYCLE_2_QUESTION_4 = createScoringQuestion("Cycle 2");
        CYCLE_2_QUESTION_5 = createFeederQuestion("Cycle 2");

        CYCLE_3_QUESTION_1 = createReefFaceQuestion("Cycle 3");
        CYCLE_3_QUESTION_2 = createBranchQuestion("Cycle 3");
        CYCLE_3_QUESTION_3 = createAlgaeQuestion("Cycle 3");
        CYCLE_3_QUESTION_4 = createScoringQuestion("Cycle 3");
        CYCLE_3_QUESTION_5 = createFeederQuestion("Cycle 3");
    }

    private LoggedDashboardChooser<ReefFace> createReefFaceQuestion(String cycleName) {
        final LoggedDashboardChooser<ReefFace> question = new LoggedDashboardChooser<>(cycleName + ", Which Reef Face?");

        for (ReefFace face : ReefFace.values()) {
            question.addOption("Face " + face.ordinal(), face);
        }

        return question;
    }

    private LoggedDashboardChooser<String> createBranchQuestion(String cycleName) {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>(cycleName + ", Which Branch?");

        question.addOption("Left Branch", "LEFT");
        question.addOption("Right Branch", "RIGHT");

        return question;
    }

    private LoggedDashboardChooser<Command> createAlgaeQuestion(String cycleName) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleName + ", Should Remove Algae?");

        question.addOption("Yes", AlgaeManipulationCommands.blastAlgaeOffReef());
        question.addOption("No", Commands.none());

        return question;
    }

    private LoggedDashboardChooser<Command> createScoringQuestion(String cycleName) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleName + ", Which Scoring Level?");

        question.addOption("L1", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L1));
        question.addOption("L2", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L2));
        question.addOption("L3", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L3));

        return question;
    }

    private LoggedDashboardChooser<Command> createFeederQuestion(String cycleName) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleName + ", Which Feeder?");

        question.addOption("Top Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.TOP_FEEDER));
        question.addOption("Bottom Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.BOTTOM_FEEDER));

        return question;
    }

    private Command createCycle(LoggedDashboardChooser<ReefFace> reefFaceQuestion,
                                LoggedDashboardChooser<String> branchQuestion,
                                LoggedDashboardChooser<Command> algaeQuestion,
                                LoggedDashboardChooser<Command> scoringHeightQuestion,
                                LoggedDashboardChooser<Command> feederQuestion) {
        final ReefFace selectedReefFace = reefFaceQuestion.get();
        final String selectedBranch = branchQuestion.get();

        Command goToBranch = selectedBranch.equals("LEFT") ?
                PathfindingCommands.pathfindToLeftBranch(selectedReefFace) :
                PathfindingCommands.pathfindToRightBranch(selectedReefFace);

        return Commands.sequence(
                goToBranch,
                algaeQuestion.get(),
                scoringHeightQuestion.get(),
                feederQuestion.get()
        );
    }

    public Command getCommand() {
        return Commands.sequence(
                createCycle(CYCLE_1_QUESTION_1, CYCLE_1_QUESTION_2, CYCLE_1_QUESTION_3, CYCLE_1_QUESTION_4, CYCLE_1_QUESTION_5),
                createCycle(CYCLE_2_QUESTION_1, CYCLE_2_QUESTION_2, CYCLE_2_QUESTION_3, CYCLE_2_QUESTION_4, CYCLE_2_QUESTION_5),
                createCycle(CYCLE_3_QUESTION_1, CYCLE_3_QUESTION_2, CYCLE_3_QUESTION_3, CYCLE_3_QUESTION_4, CYCLE_3_QUESTION_5)
        );
    }

    public String getSelected() {
        return "None";
    }
}