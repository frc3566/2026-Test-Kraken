package frc.robot.commands.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;

public class TagUtil {
    public enum LeftRight {
        LEFT, RIGHT
    }

    // Corresponding tag pairs. First is blue, second is red.
    public enum Hub {
        // FRONT = Facing the alliance, CENTER/LEFT = position on the face
        FRONT_CENTER(new Pair<Integer, Integer>(10, 26)), 
        FRONT_LEFT(new Pair<Integer, Integer>(9, 25)),
        LEFT_CENTER(new Pair<Integer, Integer>(5, 21)),
        LEFT_RIGHT(new Pair<Integer, Integer>(8, 24)),
        RIGHT_CENTER(new Pair<Integer, Integer>(2, 18)),
        RIGHT_LEFT(new Pair<Integer, Integer>(11, 27)),
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
}