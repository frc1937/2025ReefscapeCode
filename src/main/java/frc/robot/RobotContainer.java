package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.Questionnaire;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.algaeblaster.AlgaeBlaster;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.PathPlannerConstants;

import static frc.robot.poseestimation.apriltagcamera.AprilTagCameraConstants.*;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            FRONT_LEFT_CAMERA,
            FRONT_RIGHT_CAMERA,
            REAR_LEFT_CAMERA,
            REAR_RIGHT_CAMERA
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Elevator ELEVATOR = new Elevator();
    public static final CoralIntake CORAL_INTAKE = new CoralIntake();
    public static final AlgaeBlaster ALGAE_BLASTER = new AlgaeBlaster();
    public static final Climb CLIMB = new Climb();
    public static final Leds LEDS = new Leds();
    public static final Questionnaire QUESTIONNAIRE = new Questionnaire();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
        PathPlannerConstants.initializePathPlanner();

        setupLEDs();

        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TELEOP);
    }

    public Command getAutonomousCommand() {
        return QUESTIONNAIRE.getCommand();
    }

    public String getAutoName() {
        return QUESTIONNAIRE.getSelected();
    }

    private void setupLEDs() {
        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        final Trigger batteryLowTrigger = new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7)
                lowBatteryCounter[0]++;

            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        });

        batteryLowTrigger.onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 5));
    }

    public void updateComponentPoses() {
        ALGAE_BLASTER.printPose();
        ELEVATOR.printPose();
    }
}