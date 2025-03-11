package frc.robot.commands.autocommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.flippable.FlippablePose2d;
import frc.robot.commands.pathfinding.PathfindingCommands;
import frc.robot.commands.pathfinding.PathfindingConstants.Branch;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.FieldConstants.Feeder;
import frc.robot.utilities.FieldConstants.ReefFace;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.RobotContainer.CORAL_INTAKE;
import static frc.robot.RobotContainer.ELEVATOR;
import static frc.robot.commands.AlgaeManipulationCommands.blastAlgaeOffReef;
import static frc.robot.commands.CoralManipulationCommands.pathfindToFeederAndEat;
import static frc.robot.commands.CoralManipulationCommands.scoreCoralFromHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;

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

        question.addDefaultOption("None", "None");
        question.addOption("L2x3", "L2x3");
        question.addOption("TryL2", "TryL2");

        return question;
    }

    private LoggedDashboardChooser<ReefFace> createReefFaceQuestion(String cycleNumber) {
        final LoggedDashboardChooser<ReefFace> question = new LoggedDashboardChooser<>(cycleNumber + "Which Reef Face?");

        question.addDefaultOption("None", ReefFace.FACE_0);

        for (ReefFace face : ReefFace.values()) {
            question.addOption("Face " + face.ordinal(), face);
        }

//        question.addOption("Face 0", ReefFace.FACE_0);
//        question.addOption("Face 2", ReefFace.FACE_2);
//        question.addOption("Face 4", ReefFace.FACE_4);

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

        emptyCommand.schedule();

        return question;
    }

    private LoggedDashboardChooser<ElevatorConstants.ElevatorHeight> createScoringQuestion(String cycleNumber) {
        final LoggedDashboardChooser<ElevatorConstants.ElevatorHeight> question = new LoggedDashboardChooser<>(cycleNumber + "Which Scoring Level?");

        question.addDefaultOption("L2", L2);
        question.addOption("L1", L1);
        question.addOption("L3", L3);

        return question;
    }

    private LoggedDashboardChooser<Command> createFeederQuestion(String cycleNumber) {
        final LoggedDashboardChooser<Command> question = new LoggedDashboardChooser<>(cycleNumber + "Which Feeder?");

        question.addDefaultOption("None", Commands.none());
        question.addOption("Blue's left Red's right Feeder", pathfindToFeederAndEat(Feeder.BLUES_LEFT_REDS_RIGHT_FEEDER));
        question.addOption("Blue's right Red's left Feeder", pathfindToFeederAndEat(Feeder.BLUES_RIGHT_REDS_LEFT_FEEDER));

        return question;
    }

    private Command createCycleSequence(Cycle cycle) {
        final ReefFace selectedReefFace = cycle.reefFaceQuestion.get();
        final Branch selectedBranch = cycle.branchQuestion.get();
        final ElevatorConstants.ElevatorHeight scoringHeight = cycle.scoringHeightQuestion.get();

        final Command goToBranch = (selectedBranch == null || selectedReefFace == null)
                ? Commands.none()
                : PathfindingCommands.pathfindToBranchBezier(selectedBranch, selectedReefFace);

        final Command readyTheElevator = (scoringHeight == L2 || scoringHeight == L3)
                ? ELEVATOR.setTargetHeight(L2)
                : ELEVATOR.setTargetHeight(L1);

        return goToBranch.alongWith(readyTheElevator)
                .andThen(scoreCoralFromHeight(scoringHeight))
                .andThen((cycle.feederQuestion.get()));
    }

    public Command getCommand() {
        if (PRESET_QUESTION.getSendableChooser().getSelected() == "L2x3") {
            final PathPlannerAuto followAutoPreset = new PathPlannerAuto(PRESET_QUESTION.get());
            final Command correctStartPose = SwerveCommands.goToPoseTrapezoidal(new FlippablePose2d(followAutoPreset.getStartingPose(), true).get(), 0.02, 0.5);
            return correctStartPose.andThen(followAutoPreset);
        }

        if (PRESET_QUESTION.getSendableChooser().getSelected() == "TryL2") {
            return SwerveCommands.driveOpenLoop(() -> 0.2, () -> 0, () -> 0, () -> true)
                    .withTimeout(8)
                    .andThen(SwerveCommands.driveOpenLoop(() -> 0, () -> 0, () -> 0, () -> true))
                    .raceWith(new WaitCommand(0.5))
                    .andThen(new WaitCommand(2))
                    .andThen(CORAL_INTAKE.releaseGamePiece());
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
            LoggedDashboardChooser<ElevatorConstants.ElevatorHeight> scoringHeightQuestion,
            LoggedDashboardChooser<Command> feederQuestion) {
    }
}