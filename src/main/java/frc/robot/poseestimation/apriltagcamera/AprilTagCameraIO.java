package frc.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.GlobalConstants;
import frc.robot.poseestimation.apriltagcamera.io.AprilTagPhotonCameraIO;
import frc.robot.poseestimation.apriltagcamera.io.AprilTagSimulationCameraIO;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.GlobalConstants.IS_SIMULATION;

public class AprilTagCameraIO {
    static AprilTagCameraIO createCamera(String name, Transform3d robotToCamera) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new AprilTagCameraIO();
        if (IS_SIMULATION)
            return new AprilTagSimulationCameraIO(name, robotToCamera);

        return new AprilTagPhotonCameraIO(name, robotToCamera);
    }

    protected void refreshInputs(AprilTagCameraInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class AprilTagCameraInputs {
        public boolean hasResult = false;
        public double latestResultTimestampSeconds = 0;
        public Pose3d bestCameraSolvePNPPose = new Pose3d();
        public Pose3d alternateCameraSolvePNPPose = new Pose3d();
        public int[] visibleTagIDs = new int[0];
        public double poseAmbiguity = 1;
        public double[] distancesFromTags = new double[0];
    }
}
