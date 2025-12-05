package frc.robot.poseestimation.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class QuestIO {
    public static QuestIO generateQuest(Transform2d robotToQuest) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY)
            return new QuestIO();

        return new QuestReal(robotToQuest);
    }

    public void setQuestFieldPose(Pose2d pose2d) {}

    public void updateInputs(QuestInputs inputs) { }
}
