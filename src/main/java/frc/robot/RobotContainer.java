package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.util.Controller;
import frc.robot.poseestimation.objectdetection.DetectionCameraFactory;
import frc.robot.poseestimation.objectdetection.DetectionCameraIO;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            FRONT_CAMERA
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Leds LEDS = new Leds();

    private final Trigger userButton = new Trigger(RobotController::getUserButton);

    private final Controller driveController = new Controller(0);

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        setupAutonomous();
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public String getAutoName() {
        return autoChooser.getSendableChooser().getSelected();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        setupLEDs();

        configureButtons(ButtonLayout.TELEOP);
    }

    private void configureButtons(ButtonLayout layout) {
        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
        }
    }

    private void setupCharacterization(GenericSubsystem subsystem) {
        driveController.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        driveController.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private enum ButtonLayout {
        TELEOP,
        CHARACTERIZE_FLYWHEEL,
        CHARACTERIZE_ARM
    }

    private void setupLEDs() {
        LEDS.setDefaultCommand(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0));

        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7) lowBatteryCounter[0]++;
            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        }).onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 5));
    }

    private void setupDriving(DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier) {
        SWERVE.setDefaultCommand(
                SwerveCommands.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,

                        () -> -driveController.getRawAxis(Controller.Axis.RIGHT_X) * 6,
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }

    private void configureButtonsTeleop() {
        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        setupDriving(translationSupplier, strafeSupplier);

//        driveController.getButton(Controller.Inputs.A)
//            .whileTrue(SwerveCommands.driveAndRotateToClosestNote(translationSupplier, strafeSupplier));


//        driveController.getButton(Controller.Inputs.B).whileTrue(
//                new WheelRadiusCharacterization(
//                        SWERVE,
//                        MODULE_LOCATIONS,
//                        SWERVE::getDriveWheelPositionsRadians,
//                        () -> SWERVE.getGyroHeading().getRadians(),
//                        SWERVE::runWheelCharacterization
//                ));
//        );
    }

    private void setupAutonomous() {
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();

        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser(""));
    }
}
