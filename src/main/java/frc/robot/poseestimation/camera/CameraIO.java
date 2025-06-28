package frc.robot.poseestimation.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class CameraIO {
    public static CameraIO generateCamera(String name, Transform3d cameraToRobot) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new CameraIO();

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
//            return new CameraPhotonSimulation(name, cameraToRobot);
        }

        return new CameraPhotonReal(name, cameraToRobot);
    }

    public void updateInputs(CameraIOInputsAutoLogged inputs) {}

    @AutoLog
    public static class CameraIOInputs {
        public boolean hasResult;

        public double timestamp;
        public double estimatedDistanceFromTag;

        public Pose3d estimatedPose;
    }
}
