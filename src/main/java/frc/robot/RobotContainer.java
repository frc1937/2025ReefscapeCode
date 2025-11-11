package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.Questionnaire;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.poseestimation.quest.Quest;
import frc.robot.subsystems.algaeblaster.AlgaeBlaster;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.coralintake.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utilities.PathPlannerConstants;

import static edu.wpi.first.math.util.Units.degreesToRadians;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final Quest QUEST = new Quest(
            new Transform2d(new Translation2d(-0.035, 0.22), Rotation2d.fromDegrees(0)));

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{
                    new Camera("FRONT_LEFT_CAMERA", new Transform3d(
                            0.24, 0.29, 0.21,
                            new Rotation3d(0, degreesToRadians(-25.212676878873813), degreesToRadians(330))).inverse()),
            },

            QUEST

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

        setupLEDsForBattery();

        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TELEOP);
    }

    public Command getAutonomousCommand() {
        return QUESTIONNAIRE.getCommand();
    }

    public String getAutoName() {
        return QUESTIONNAIRE.getSelected();
    }

    private void setupLEDsForBattery() {
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