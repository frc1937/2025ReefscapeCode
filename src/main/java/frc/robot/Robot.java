package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LoggedRobot;

import java.io.IOException;
import java.util.Optional;

import static frc.robot.RobotContainer.LEDS;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.poseestimation.photoncamera.VisionConstants.VISION_SIMULATION;

public class Robot extends LoggedRobot {
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
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
        try {
            if (robotContainer.getAutoName().equals("None")) return;

            final PathPlannerPath path = PathPlannerPath.fromPathFile(robotContainer.getAutoName());
            final Optional<Pose2d> startingPose = path.getStartingHolonomicPose();

            if (startingPose.isEmpty()) return;

            LEDS.setLEDToPositionIndicator(
                    POSE_ESTIMATOR.getCurrentPose().getTranslation(),
                    startingPose.get().getTranslation()
            );

        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
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
        VISION_SIMULATION.updateRobotPose(POSE_ESTIMATOR.getOdometryPose());
    }
}