package frc.robot.commands.vision;

import edu.wpi.first.math.Pair;
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
            return new Translation2d(4.625, 4.03);
        } else {
            return new Translation2d(11.915, 4.03);

        }
    }

    public static Translation2d getLeftTrenchTranslation(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Translation2d(3.0, 7.375);
        } else {
            return new Translation2d(13.5, 0.625);
        }
    }

    public static Translation2d getRightTrenchTranslation(){
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return new Translation2d(3.0, 0.625);
        } else {
            return new Translation2d(13.5, 7.375);
        }
    }



    public static Translation2d getHubFrontCenterTagTranslation(){
        var centerId = Hub.FRONT_CENTER.getTargettingId();
        return Vision.fieldLayout.getTagPose(centerId).get().getTranslation().toTranslation2d();
    }


}