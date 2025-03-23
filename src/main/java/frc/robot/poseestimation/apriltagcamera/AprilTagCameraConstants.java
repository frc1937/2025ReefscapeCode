package frc.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.poseestimation.poseestimator.StandardDeviations;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.GlobalConstants.IS_SIMULATION;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.APRIL_TAG_FIELD_LAYOUT;

public class AprilTagCameraConstants {
    static final double MAXIMUM_DISTANCE_FROM_TAG_FOR_ACCURATE_SOLVE_PNP_RESULT_METERS = 2;
    static final Pose2d[] EMPTY_POSE_LIST = new Pose2d[0];
    static final double MAXIMUM_AMBIGUITY = 0.4;

    public static final VisionSystemSim VISION_SIMULATION = IS_SIMULATION ? new VisionSystemSim("VisionSimulation") : null;
    private static final int
            SIMULATION_CAMERA_RESOLUTION_WIDTH = 1600,
            SIMULATION_CAMERA_RESOLUTION_HEIGHT = 1200,
            SIMULATION_CAMERA_FPS = 60,
            SIMULATION_AVERAGE_CAMERA_LATENCY_MILLISECONDS = 35,
            SIMULATION_CAMERA_LATENCY_STANDARD_DEVIATIONS_MILLISECONDS = 5,
            SIMULATION_CAMERA_EXPOSURE_TIME_MILLISECONDS = 10;
    private static final Rotation2d SIMULATION_CAMERA_FOV = Rotation2d.fromDegrees(70);
    private static final double
            SIMULATION_CAMERA_AVERAGE_PIXEL_ERROR = 0.25,
            SIMULATION_CAMERA_PIXEL_STANDARD_DEVIATIONS = 0.08;
    public static final SimCameraProperties SIMULATION_CAMERA_PROPERTIES = new SimCameraProperties();

    static {
        if (IS_SIMULATION) {
            configureSimulationCameraProperties();
            VISION_SIMULATION.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
        }
    }

    private static void configureSimulationCameraProperties() {
        SIMULATION_CAMERA_PROPERTIES.setCalibration(SIMULATION_CAMERA_RESOLUTION_WIDTH, SIMULATION_CAMERA_RESOLUTION_HEIGHT, SIMULATION_CAMERA_FOV);
        SIMULATION_CAMERA_PROPERTIES.setCalibError(SIMULATION_CAMERA_AVERAGE_PIXEL_ERROR, SIMULATION_CAMERA_PIXEL_STANDARD_DEVIATIONS);
        SIMULATION_CAMERA_PROPERTIES.setFPS(SIMULATION_CAMERA_FPS);
        SIMULATION_CAMERA_PROPERTIES.setAvgLatencyMs(SIMULATION_AVERAGE_CAMERA_LATENCY_MILLISECONDS);
        SIMULATION_CAMERA_PROPERTIES.setLatencyStdDevMs(SIMULATION_CAMERA_LATENCY_STANDARD_DEVIATIONS_MILLISECONDS);
        SIMULATION_CAMERA_PROPERTIES.setExposureTimeMs(SIMULATION_CAMERA_EXPOSURE_TIME_MILLISECONDS);
    }

    private static final StandardDeviations
            REEF_TAG_CAMERA_STANDARD_DEVIATIONS
            = new StandardDeviations(
            0.015,
            0.01
    );

    public static final Transform3d
            ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
                    0.24, 0.29,0.21,
                    new Rotation3d(0, degreesToRadians(-25.212676878873813), degreesToRadians(330))),

            ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
                    0.24,-0.29,0.21,
                    new Rotation3d(0, degreesToRadians(-25.212676878873813), degreesToRadians(30.3))),

            ROBOT_TO_REAR_LEFT_CAMERA = new Transform3d(
                    -0.24, 0.275,0.21,
                    new Rotation3d(0, degreesToRadians(-27.516516626188296),
                            degreesToRadians(150))),

            ROBOT_TO_REAR_RIGHT_CAMERA = new Transform3d(
                    -0.24,-0.275,0.21,
                    new Rotation3d(0, degreesToRadians(-28.809065526902253),
                            degreesToRadians(210))
            );

    public static final AprilTagCamera
            FRONT_LEFT_CAMERA = new AprilTagCamera("FRONT_LEFT", ROBOT_TO_FRONT_LEFT_CAMERA, REEF_TAG_CAMERA_STANDARD_DEVIATIONS),
            FRONT_RIGHT_CAMERA = new AprilTagCamera("FRONT_RIGHT", ROBOT_TO_FRONT_RIGHT_CAMERA, new StandardDeviations(0.007, 0.007)),
            REAR_LEFT_CAMERA = new AprilTagCamera("REAR_LEFT", ROBOT_TO_REAR_LEFT_CAMERA, REEF_TAG_CAMERA_STANDARD_DEVIATIONS),
            REAR_RIGHT_CAMERA = new AprilTagCamera("REAR_RIGHT", ROBOT_TO_REAR_RIGHT_CAMERA, REEF_TAG_CAMERA_STANDARD_DEVIATIONS);


}
