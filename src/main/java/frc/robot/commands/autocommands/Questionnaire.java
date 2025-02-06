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

import static frc.robot.RobotContainer.CORAL_INTAKE;

public class Questionnaire {
    private final LoggedDashboardChooser<Command> PRESET_QUESTION;
    private final Cycle
            CYCLE_1,
            CYCLE_2,
            CYCLE_3;

    public Questionnaire() {
        PRESET_QUESTION = createPresetQuestion();

        CYCLE_1 = initializeCycleFromKey("Cycle1/");
        CYCLE_2 = initializeCycleFromKey("Cycle2/");
        CYCLE_3 = initializeCycleFromKey("Cycle3/");
    }

    private Cycle initializeCycleFromKey(String key) {
        return new Cycle(createReefFaceQuestion(key),
                createBranchQuestion(key),
                createAlgaeQuestion(key),
                createScoringQuestion(key),
                createFeederQuestion(key));
    }

    private LoggedDashboardChooser<Command> createPresetQuestion() {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>("Which Auto Preset?");

        question.addDefaultOption("None", Commands.none());
        question.addOption("go and distract on other alliance", Commands.none());
        question.addOption("run into a wall", Commands.none());
        question.addOption("spin around the reef", Commands.none());

        return question;
    }

    private LoggedDashboardChooser<String> createReefFaceQuestion(String cycleNumber) {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>(cycleNumber + "Which Reef Face?");

        question.addDefaultOption("None", "None");
        for (ReefFace face : ReefFace.values()) {
            question.addOption("Face " + face.ordinal(), face.name());
        }

        return question;
    }

    private LoggedDashboardChooser<String> createBranchQuestion(String cycleNumber) {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>(cycleNumber + "Which Branch?");

        question.addDefaultOption("None", "None");
        question.addOption("Left Branch", "LEFT");
        question.addOption("Right Branch", "RIGHT");

        return question;
    }

    private LoggedDashboardChooser<Command> createAlgaeQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Should Remove Algae?");

        question.addDefaultOption("None", Commands.none());
        question.addOption("Yes", AlgaeManipulationCommands.blastAlgaeOffReef());
        question.addOption("No", Commands.none());

        return question;
    }

    private LoggedDashboardChooser<Command> createScoringQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Which Scoring Level?");

        question.addDefaultOption("None", Commands.none());
        question.addOption("L1", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L1));
        question.addOption("L2", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L2));
        question.addOption("L3", CoralManipulationCommands.scoreGamePiece(ElevatorConstants.ElevatorHeight.L3));

        return question;
    }

    private LoggedDashboardChooser<Command> createFeederQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Which Feeder?");

        question.addDefaultOption("None", Commands.none());
        question.addOption("Top Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.TOP_FEEDER));
        question.addOption("Bottom Feeder", CoralManipulationCommands.pathfindToFeederAndEat(Feeder.BOTTOM_FEEDER));

        return question;
    }

    private Command createCycleSequence(Cycle cycle) {
        final String selectedReefFace = cycle.reefFaceQuestion.get();
        final String selectedBranch = cycle.branchQuestion.get();

        final Command goToBranch = selectedBranch.equals("None") || selectedReefFace.equals("None") ?
                Commands.none() :
                selectedBranch.equals("LEFT") ?
                        PathfindingCommands.pathfindToLeftBranch(ReefFace.valueOf(selectedReefFace)) :
                        PathfindingCommands.pathfindToRightBranch(ReefFace.valueOf(selectedReefFace));

        return Commands.sequence(
                goToBranch,
                cycle.algaeQuestion.get(),
                cycle.scoringHeightQuestion.get(),
                cycle.feederQuestion.get()
        );
    }

    public Command getCommand() {
        return PRESET_QUESTION.getSendableChooser().getSelected().equals("None") ?
                Commands.sequence(
                        createCycleSequence(CYCLE_1),
                        createCycleSequence(CYCLE_2),
                        createCycleSequence(CYCLE_3)
                ) :
                PRESET_QUESTION.get();
    }

    public String getSelected() {
        return "None";
    }

    private record Cycle(
            LoggedDashboardChooser<String> reefFaceQuestion,
            LoggedDashboardChooser<String> branchQuestion,
            LoggedDashboardChooser<Command> algaeQuestion,
            LoggedDashboardChooser<Command> scoringHeightQuestion,
            LoggedDashboardChooser<Command> feederQuestion) {
    }
}