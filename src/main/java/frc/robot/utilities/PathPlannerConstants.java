package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.util.LocalADStarAK;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.CoralManipulationCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static frc.robot.GlobalConstants.IS_SIMULATION;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;

public class PathPlannerConstants {
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();

    public static final PathConstraints PATHPLANNER_CONSTRAINTS = IS_SIMULATION
            ? new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 2, 6, 4)
            : new PathConstraints(SwerveConstants.MAX_SPEED_MPS, 3.3, Math.PI*1.3, Math.PI*1.3);

    public static final PPHolonomicDriveController PATHPLANNER_DRIVE_CONTROLLER = IS_SIMULATION
            ? new PPHolonomicDriveController(new PIDConstants(4.5, 0.0, 0), new PIDConstants(0.9, 0.0, 0))
            : new PPHolonomicDriveController(new PIDConstants(2.75, 0.0, 0), new PIDConstants(1.77, 0.0, 0));

    public static void initializePathPlanner() {
        Pathfinding.setPathfinder(new LocalADStarAK());

        configurePathPlanner();
        setupNamedCommands();

        PathfindingCommand.warmupCommand().schedule();
    }

    private static void setupNamedCommands() {
        NamedCommands.registerCommand("ScoreCoralL2", CoralManipulationCommands.scoreCoralFromHeight(ElevatorConstants.ElevatorHeight.L2));
        NamedCommands.registerCommand("EatFromFeeder", CoralManipulationCommands.eatFromFeeder());
    }

    private static void configurePathPlanner() {
        AutoBuilder.configure(
                POSE_ESTIMATOR::getCurrentPose,
                POSE_ESTIMATOR::resetPose,
                SWERVE::getRobotRelativeVelocity,
                (ChassisSpeeds speeds) -> SWERVE.driveRobotRelative(speeds, true),
                PATHPLANNER_DRIVE_CONTROLLER,
                ROBOT_CONFIG,
                Flippable::isRedAlliance,
                SWERVE
        );
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
