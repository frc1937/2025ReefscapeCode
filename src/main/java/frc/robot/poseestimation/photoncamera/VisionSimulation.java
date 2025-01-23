package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.GlobalConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class VisionSimulation {
    private VisionSystemSim visionSystemSim;
    private SimCameraProperties properties;

    /**
     * Won't do anything if not in SIM
     */
    public void initializeVisionSimulation() {
        if (GlobalConstants.Mode.REAL == CURRENT_MODE)
            return;

        visionSystemSim = new VisionSystemSim("main");
        properties = new SimCameraProperties();

        visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
        setupCameraProperties();
    }

    /**
     * Throws an exception if the mode is not SIM
     * @param camera The camera to add
     * @param robotCenterToCamera The transform from the robot center to the camera
     */
    public void addCamera(PhotonCamera camera, Transform3d robotCenterToCamera) {
        final PhotonCameraSim simulatedCamera = new PhotonCameraSim(camera, properties);

        visionSystemSim.addCamera(simulatedCamera, robotCenterToCamera);

        simulatedCamera.enableDrawWireframe(true);
    }

    /**
     * Throws an exception if the mode is not SIM
     * @param pose The pose to update the VisionSym field with.
     */
    public void updateRobotPose(Pose2d pose) {
        visionSystemSim.update(pose);
    }

    private void setupCameraProperties() {
        properties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        properties.setCalibError(0.12, 0.10);
        properties.setFPS(15);
        properties.setAvgLatencyMs(50);
        properties.setLatencyStdDevMs(15);
    }
}
