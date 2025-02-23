package frc.robot.commands.autocommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.flippable.FlippablePose2d;
import frc.robot.commands.CoralManipulationCommands;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.commands.pathfinding.PathfindingConstants.Branch;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.FieldConstants.Feeder;
import frc.robot.utilities.FieldConstants.ReefFace;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.commands.AlgaeManipulationCommands.blastAlgaeOffReef;

public class Questionnaire {
    private final LoggedDashboardChooser<String> PRESET_QUESTION;
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

    private LoggedDashboardChooser<String> createPresetQuestion() {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>("Which Auto Preset?");

        question.addDefaultOption("None", "none");
        question.addOption("L2x3", "L2x3");

        return question;
    }

    private LoggedDashboardChooser<ReefFace> createReefFaceQuestion(String cycleNumber) {
        final LoggedDashboardChooser<ReefFace> question = new LoggedDashboardChooser<>(cycleNumber + "Which Reef Face?");

        question.addDefaultOption("None", null);

        for (ReefFace face : ReefFace.values()) {
            question.addOption("Face " + face.ordinal(), face.getAllianceCorrectedFace());
        }

        return question;
    }

    private LoggedDashboardChooser<Branch> createBranchQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Branch> question = new LoggedDashboardChooser<>(cycleNumber + "Which Branch?");

        question.addDefaultOption("None", null);
        question.addOption("Left Branch", Branch.LEFT_BRANCH);
        question.addOption("Right Branch", Branch.RIGHT_BRANCH);

        return question;
    }

    private LoggedDashboardChooser<Command> createAlgaeQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Should Remove Algae?");

        final Command emptyCommand = Commands.none();

        question.addDefaultOption("None", emptyCommand);
        question.addOption("Yes", blastAlgaeOffReef());
        question.addOption("No", emptyCommand);

        emptyCommand.schedule();

        return question;
    }

    private LoggedDashboardChooser<Command> createScoringQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Which Scoring Level?");

        question.addDefaultOption("None", Commands.none());
        question.addOption("L1", CoralManipulationCommands.scoreCoralFromHeight(ElevatorConstants.ElevatorHeight.L1));
        question.addOption("L2", CoralManipulationCommands.scoreCoralFromHeight(ElevatorConstants.ElevatorHeight.L2));
        question.addOption("L3", CoralManipulationCommands.scoreCoralFromHeight(ElevatorConstants.ElevatorHeight.L3));

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
        final ReefFace selectedReefFace = cycle.reefFaceQuestion.get();
        final Branch selectedBranch = cycle.branchQuestion.get();

        final Command goToBranch = selectedBranch == null || selectedReefFace == null
                ? Commands.none()
                : PathfindingCommands.pathfindToBranchBezier(selectedBranch, selectedReefFace);

        final Command algaeBlastingCommand = cycle.algaeQuestion.get().isFinished()
                ? Commands.none()
                : blastAlgaeOffReef(selectedReefFace);

        return goToBranch
                .alongWith(algaeBlastingCommand)
                .andThen(cycle.scoringHeightQuestion.get())
                .andThen((cycle.coralIntakeQuestion.get()));
    }

    public Command getCommand() {
        if (PRESET_QUESTION.getSendableChooser().getSelected() != "None") {
            final PathPlannerAuto presetAutoPath = new PathPlannerAuto(PRESET_QUESTION.get());
            return SwerveCommands.goToPoseBezier(new FlippablePose2d(presetAutoPath.getStartingPose(), true).get()).andThen(presetAutoPath);
        }

        return Commands.sequence(
                createCycleSequence(CYCLE_1),
                createCycleSequence(CYCLE_2),
                createCycleSequence(CYCLE_3)
        );
    }

    public String getSelected() {
        return PRESET_QUESTION.getSendableChooser().getSelected() != "None" ? PRESET_QUESTION.get() : "Custom";
    }

    private record Cycle(
            LoggedDashboardChooser<ReefFace> reefFaceQuestion,
            LoggedDashboardChooser<Branch> branchQuestion,
            LoggedDashboardChooser<Command> algaeQuestion,
            LoggedDashboardChooser<Command> scoringHeightQuestion,
            LoggedDashboardChooser<Command> coralIntakeQuestion) {
    }
}