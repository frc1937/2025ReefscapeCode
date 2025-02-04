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
    private final Cycle
            CYCLE_1,
            CYCLE_2,
            CYCLE_3;

    public Questionnaire() {
        CYCLE_1 = new Cycle(
                createReefFaceQuestion("Cycle 1"),
                createBranchQuestion("Cycle 1"),
                createAlgaeQuestion("Cycle 1"),
                createScoringQuestion("Cycle 1"),
                createFeederQuestion("Cycle 1")
        );

        CYCLE_2 = new Cycle(createReefFaceQuestion("Cycle 2"),
                createBranchQuestion("Cycle 2"),
                createAlgaeQuestion("Cycle 2"),
                createScoringQuestion("Cycle 2"),
                createFeederQuestion("Cycle 2"));

        CYCLE_3 = new Cycle(createReefFaceQuestion("Cycle 3"),
                createBranchQuestion("Cycle 3"),
                createAlgaeQuestion("Cycle 3"),
                createScoringQuestion("Cycle 3"),
                createFeederQuestion("Cycle 3"));
    }

    private LoggedDashboardChooser<ReefFace> createReefFaceQuestion(String cycleNumber) {
        final LoggedDashboardChooser<ReefFace> question = new LoggedDashboardChooser<>(cycleNumber + ", Which Reef Face?");

        for (ReefFace face : ReefFace.values()) {
            question.addOption("Face " + face.ordinal(), face);
        }

        return question;
    }

    private LoggedDashboardChooser<String> createBranchQuestion(String cycleNumber) {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>(cycleNumber + ", Which Branch?");

        question.addOption("Left Branch", "LEFT");
        question.addOption("Right Branch", "RIGHT");

        return question;
    }

    private LoggedDashboardChooser<Command> createAlgaeQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + ", Should Remove Algae?");

        question.addOption("Yes", AlgaeManipulationCommands.blastAlgaeOffReef());
        question.addOption("No", Commands.none());

        return question;
    }

    private LoggedDashboardChooser<Command> createScoringQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + ", Which Scoring Level?");

        question.addOption("L1", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L1));
        question.addOption("L2", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L2));
        question.addOption("L3", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L3));

        return question;
    }

    private LoggedDashboardChooser<Command> createFeederQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + ", Which Feeder?");

        question.addOption("Top Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.TOP_FEEDER));
        question.addOption("Bottom Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.BOTTOM_FEEDER));

        return question;
    }

    private Command createCycleSequence(Cycle cycle) {
        final ReefFace selectedReefFace = cycle.reefFaceQuestion.get();
        final String selectedBranch = cycle.branchQuestion.get();

        final Command goToBranch = selectedBranch.equals("LEFT") ?
                PathfindingCommands.pathfindToLeftBranch(selectedReefFace) :
                PathfindingCommands.pathfindToRightBranch(selectedReefFace);

        return Commands.sequence(
                goToBranch,
                cycle.algaeQuestion.get(),
                cycle.scoringHeightQuestion.get(),
                cycle.feederQuestion.get()
        );
    }

    public Command getCommand() {
        return Commands.sequence(
                createCycleSequence(CYCLE_1),
                createCycleSequence(CYCLE_2),
                createCycleSequence(CYCLE_3)
        );
    }

    public String getSelected() {
        return "None";
    }

    private record Cycle(
            LoggedDashboardChooser<ReefFace> reefFaceQuestion,
            LoggedDashboardChooser<String> branchQuestion,
            LoggedDashboardChooser<Command> algaeQuestion,
            LoggedDashboardChooser<Command> scoringHeightQuestion,
            LoggedDashboardChooser<Command> feederQuestion) {
    }
}