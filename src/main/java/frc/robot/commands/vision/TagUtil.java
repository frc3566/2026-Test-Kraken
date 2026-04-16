package frc.robot.commands.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Vision;

public class TagUtil {
    public enum LeftRight {
        LEFT, RIGHT
    }

    // Corresponding tag pairs. First is blue, second is red.
    public enum Hub {
        // FRONT = Facing the alliance, CENTER/LEFT = position on the face
        FRONT_CENTER(new Pair<Integer, Integer>(26, 10)), 
        FRONT_LEFT(new Pair<Integer, Integer>(25, 9)),
        LEFT_CENTER(new Pair<Integer, Integer>(21, 5)),
        LEFT_RIGHT(new Pair<Integer, Integer>(24, 8)),
        RIGHT_CENTER(new Pair<Integer, Integer>(18, 2)),
        RIGHT_LEFT(new Pair<Integer, Integer>(27, 11)),
        ;

        private Pair<Integer, Integer> targettingIds;
        
        private Hub(Pair<Integer, Integer> targettingIds) {
            this.targettingIds = targettingIds;
        }


        /**
         *  Returns the appropriate target ID based on the current alliance color.
         * @return the target ID for the current alliance
         */
        public int getTargettingId() {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                return targettingIds.getFirst();
            } else {
                return targettingIds.getSecond();
            }
        }


    }

    // public enum BranchLevel {
    //     TROUGH, L2, L3, L4
    // }

    // public static List<Integer> getTargettingIds() {
    //     if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
    //         return List.of(17, 18, 19, 20, 21, 22);
    //     } else {
    //         return List.of(6, 7, 8, 9, 10, 11);
    //     }
    // }
    public static Translation2d getHubCenterTranslation(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Translation2d(4.611, 4.02);
        } else {
            return new Translation2d(11.901, 4.02);

        }
    }

    // Pose2d from blue alliance's perspective
    // Red front = 180 degrees
    public static Pose2d getLeftTrenchPose(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Pose2d(new Translation2d(3.0, 7.411), Rotation2d.kPi);
        } else {
            return new Pose2d(new Translation2d(13.5, 0.631), new Rotation2d(0));
        }
    }

    public static Pose2d getRightTrenchPose(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Pose2d(new Translation2d(3.0, 0.631), Rotation2d.kPi);
        } else {
            return new Pose2d(new Translation2d(13.5, 7.411), new Rotation2d(0));
        }
    }

        public static Pose2d getLeftTrenchNeutralPose(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Pose2d(new Translation2d(6.0, 7.411), Rotation2d.kPi);
        } else {
            return new Pose2d(new Translation2d(10.5, 0.631), new Rotation2d(0));
        }
    }

    public static Pose2d getRightTrenchNeutralPose(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Pose2d(new Translation2d(6.0, 0.631), Rotation2d.kPi);
        } else {
            return new Pose2d(new Translation2d(10.5, 7.411), new Rotation2d(0));
        }
    }



    public static Translation2d getHubFrontCenterTagTranslation(){
        var centerId = Hub.FRONT_CENTER.getTargettingId();
        return Vision.fieldLayout.getTagPose(centerId).get().getTranslation().toTranslation2d();
    }


}