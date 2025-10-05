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
import org.littletonrobotics.junction.Logger;

import java.io.File;
import java.io.IOException;

import static frc.lib.math.Optimizations.isColliding;
import static frc.robot.RobotContainer.*;

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

        final float xAccel = (float) ACCELEROMETER.getX();
        final float yAccel = (float) ACCELEROMETER.getY();
        final double totalAccel = Math.hypot(xAccel, yAccel) * 9.8015;

        Logger.recordOutput("Robot/Accelerometer X", xAccel);
        Logger.recordOutput("Robot/Accelerometer Y", yAccel);
        Logger.recordOutput("Robot/Accelerometer G", totalAccel);
        Logger.recordOutput("Robot/Is Colliding", isColliding());
        Logger.recordOutput("Robot/TotalDriveCurrent", SWERVE.getTotalCurrent());
    }

    @Override
    public void disabledPeriodic() {
        QUEST.setPose(POSE_ESTIMATOR.getCurrentPose());

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

        robotContainer.updateComponentPoses();
    }
}