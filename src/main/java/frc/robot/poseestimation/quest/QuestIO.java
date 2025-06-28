package frc.robot.poseestimation.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class QuestIO {
    public static QuestIO generateQuest(Transform2d robotToQuest) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new QuestIO();

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
//            return new CameraPhotonSimulation(name, cameraToRobot);
        }

        return new QuestReal(robotToQuest);
    }

    public void setQuestFieldPose(Pose2d pose2d) {}

    public void updateInputs(QuestIOInputsAutoLogged inputs) {}

    @AutoLog
    public static class QuestIOInputs {
        public boolean connected = false;
        public boolean tracking = false;

        public double timestamp = -1.0;
        public double batteryPercent = -1.0;

        public Pose2d robotPose = Pose2d.kZero;
    }
}
