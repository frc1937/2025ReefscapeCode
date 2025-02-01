package frc.robot.poseestimation.photoncamera;

public class VisionConstants {
    public static final VisionSimulation VISION_SIMULATION = new VisionSimulation();

    static {
        VISION_SIMULATION.initializeVisionSimulation();
    }
}
