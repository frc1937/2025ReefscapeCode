package frc.robot.poseestimation.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

import static frc.robot.poseestimation.PoseEstimatorConstants.*;

public class Camera {
    private final String prefix;

    private final CameraIO cameraIO;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public Camera(String name, Transform3d cameraToRobot) {
        cameraIO = CameraIO.generateCamera(name, cameraToRobot);
        prefix = "Camera/" + name;
    }

    //MUST BE CALLED PERIODICALLY
    public void refreshInputs() {
        cameraIO.updateInputs(inputs);
        Logger.processInputs(prefix, inputs);
    }

    public Pose2d getEstimatedPose() { return inputs.estimatedPose.toPose2d(); }

    public double getTimestamp() { return inputs.timestamp; }

    public boolean hasResult() {
        final boolean rejectPose = !inputs.hasResult
                || Math.abs(inputs.estimatedPose.getZ()) > MAX_Z_ERROR
                || inputs.estimatedPose.getX() < 0.0
                || inputs.estimatedPose.getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
                || inputs.estimatedPose.getY() < 0.0
                || inputs.estimatedPose.getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

        return !rejectPose;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        final double standardDeviationFactor = Math.pow(inputs.estimatedDistanceFromTag, 2.0);

        final double linearStandardDeviation = VISION_STD_LINEAR * standardDeviationFactor;
        final double angularStandardDeviation = VISION_STD_ANGULAR * standardDeviationFactor;

        return VecBuilder.fill(linearStandardDeviation, linearStandardDeviation, angularStandardDeviation);
    }
}
