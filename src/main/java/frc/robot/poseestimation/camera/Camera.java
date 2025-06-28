package frc.robot.poseestimation.camera;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;

public class Camera {
    private final String prefix;

    private final CameraIO cameraIO;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public Camera(String name, Transform3d cameraToRobot) {
        cameraIO = CameraIO.generateCamera(name, cameraToRobot);
        prefix = "Camera/" + name;
    }

    public void refreshInputs() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs(prefix, inputs);
    }

    public EstimateData[] getEstimates() { return inputs.estimations; }

    public boolean cameraHasResults() {
        return inputs.hasResult;
    }
}
