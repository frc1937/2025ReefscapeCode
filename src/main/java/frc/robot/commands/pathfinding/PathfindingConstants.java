package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.flippable.Flippable;
import frc.robot.utilities.FieldConstants;

public class PathfindingConstants {
    public enum BranchOption {
        LEFT_BRANCH {
            @Override
            public Pose2d getBranchPose() {
                return PathfindingCommands.decideReefFace().getLeftBranch();
            }

            @Override
            public Pose2d getBranchPose(FieldConstants.ReefFace face) {
                return Flippable.isRedAlliance() ?
                        face.getOpposite().getLeftBranch() :
                        face.getLeftBranch();
            }
        },

        RIGHT_BRANCH {
            @Override
            public Pose2d getBranchPose() {
                return PathfindingCommands.decideReefFace().getRightBranch();
            }

            @Override
            public Pose2d getBranchPose(FieldConstants.ReefFace face) {
                return Flippable.isRedAlliance() ?
                        face.getOpposite().getRightBranch() :
                        face.getRightBranch();
            }
        };

        public abstract Pose2d getBranchPose();
        public abstract Pose2d getBranchPose(FieldConstants.ReefFace face);
    }
}
