package frc.robot.commands.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;

public class TagUtil {
    public enum LeftRight {
        LEFT, RIGHT
    }

    // Corresponding tag pairs. First is blue, second is red.
    public enum Side {
        // FRONT = Facing the alliance, CENTER/LEFT = position on the face
        HUB_FRONT_CENTER(new Pair<>(10, 26)), 
        HUB_FRONT_LEFT(new Pair<>(9, 25)),
        HUB_LEFT_CENTER(new Pair<>(5, 21)),
        HUB_LEFT_RIGHT(new Pair<>(8, 24)),
        HUB_RIGHT_CENTER(new Pair<>(2, 18)),
        HUB_RIGHT_LEFT(new Pair<>(11, 27)),
        ;

        private Pair<Integer, Integer> targettingIds;
        
        private Side(Pair<Integer, Integer> targettingIds) {
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
}