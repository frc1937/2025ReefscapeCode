package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;

import java.io.File;
import java.io.IOException;

import static frc.robot.RobotContainer.LEDS;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.apriltagcamera.AprilTagCameraConstants.VISION_SIMULATION;

public class Robot extends LoggedRobot {
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        SignalLogger.enableAutoLogging(false);
        HardwareManager.initialize(this);
    }

    @Override
    public void robotPeriodic() {
        HardwareManager.update();
        commandScheduler.run();

        POSE_ESTIMATOR.periodic();
    }

    @Override
    public void disabledPeriodic() {
        if (!new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + robotContainer.getAutoName() + ".path").exists())
            return;

        PathPlannerPath path = null;

        try {
            path = PathPlannerPath.fromPathFile(robotContainer.getAutoName());
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }

        if (path == null) return;

        final Pose2d startingPose = path.getStartingHolonomicPose().get();

        LEDS.setLEDToPositionIndicator(
                POSE_ESTIMATOR.getCurrentPose().getTranslation(),
                startingPose.getTranslation()
        );
    }

    @Override
    public void autonomousInit() {
        final Command autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            autonomousCommand.schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();
        VISION_SIMULATION.update(POSE_ESTIMATOR.getOdometryPose());

        robotContainer.updateComponentPoses();
    }
}