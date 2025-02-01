package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.util.LocalADStarAK;
import frc.lib.util.flippable.Flippable;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;

public class PathPlannerConstants {
    public static final RobotConfig ROBOT_CONFIG = getRobotConfig();

    public static PathPlannerPath ALIGN_TO_TOP_FEEDER;
    public static PathPlannerPath ALIGN_TO_BOTTOM_FEEDER;

    static {
        try {
            ALIGN_TO_TOP_FEEDER = PathPlannerPath.fromPathFile("nextToTopFeeder");
            ALIGN_TO_BOTTOM_FEEDER = PathPlannerPath.fromPathFile("nextToBottomFeeder");
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    public static final PPHolonomicDriveController PATHPLANNER_PID_CONSTANTS = new PPHolonomicDriveController(
            new PIDConstants(5.1, 0.0, 0),
            new PIDConstants(3, 0.0, 0)
    );

    public static final PathConstraints PATHPLANNER_CONSTRAINTS = new PathConstraints(
            ROBOT_CONFIG.moduleConfig.maxDriveVelocityMPS,
            3,
            ROBOT_CONFIG.moduleConfig.maxDriveVelocityRadPerSec,
            2
    );

    public static final PIDConstants PATHPLANNER_CAGE_CONSTRAINTS = new PIDConstants(0.5);


    public static void initializePathPlanner() {
        Pathfinding.setPathfinder(new LocalADStarAK());

        configurePathPlanner();

        PathfindingCommand.warmupCommand().schedule();
    }

    private static void configurePathPlanner() {
        AutoBuilder.configure(
                POSE_ESTIMATOR::getCurrentPose,
                POSE_ESTIMATOR::resetPose,
                SWERVE::getRobotRelativeVelocity,
                SWERVE::driveRobotRelative,
                PATHPLANNER_PID_CONSTANTS,
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
