package frc.robot.commands.vision;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

/**
 * Drives the robot to a target {@link Pose2d} using PathPlanner's on-the-fly
 * pathfinding ({@link AutoBuilder#pathfindToPose}).
 *
 * PathPlanner's AD* algorithm plans an obstacle-avoiding path from the robot's
 * current position to the target pose, then follows it with the holonomic
 * drive controller configured in {@link frc.robot.subsystems.CommandSwerveDrivetrain#configureAuto}.
 *
 * Vision pose estimation is disabled for the duration of this command to
 * prevent the Kalman filter from fighting the path follower's odometry updates,
 * and re-enabled when the command ends (normally or interrupted).
 */
public class DriveToPoseNew extends Command {

    // TODO: Tune these constraints on the real robot.
    private static final double MAX_VELOCITY_MPS        = 3.4;
    private static final double MAX_ACCEL_MPS2          = 3.0;
    private static final double MAX_ANGULAR_VEL_DEG_S   = 540.0;
    private static final double MAX_ANGULAR_ACCEL_DEG_S = 720.0;
    private static final double GOAL_END_VELOCITY_MPS   = 0.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        MAX_VELOCITY_MPS,
        MAX_ACCEL_MPS2,
        Units.degreesToRadians(MAX_ANGULAR_VEL_DEG_S),
        Units.degreesToRadians(MAX_ANGULAR_ACCEL_DEG_S)
    );

    private final Vision vision;
    private final Supplier<Pose2d> targetPoseSupplier;

    /** The inner PathPlanner command, built fresh each time in initialize(). */
    private Command pathfindingCommand;

    /**
     * @param vision             The vision subsystem (pose estimation is paused while driving).
     * @param targetPoseSupplier Supplier for the target {@link Pose2d}. Evaluated once in
     *                           {@link #initialize()}, so it can be dynamic (e.g. computed
     *                           from a tag pose + offset).
     */
    public DriveToPoseNew(Vision vision, Supplier<Pose2d> targetPoseSupplier) {
        this.vision = vision;
        this.targetPoseSupplier = targetPoseSupplier;
        // NOTE: We do NOT call addRequirements here because AutoBuilder.pathfindToPose
        // returns a command that already requires the drivetrain subsystem.
        // Adding the requirement again would cause a conflict.
    }

    @Override
    public void initialize() {
        // vision.setPoseEstimationEnabled(false);

        Pose2d targetPose = targetPoseSupplier.get();
        SmartDashboard.putString("DriveToPoseNew/Target Pose", targetPose.toString());
        System.out.println("DriveToPoseNew: pathfinding to " + targetPose);

        pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS, GOAL_END_VELOCITY_MPS);
        pathfindingCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathfindingCommand != null) {
            pathfindingCommand.execute();
        }
        SmartDashboard.putBoolean("DriveToPoseNew/Running", true);
    }

    @Override
    public boolean isFinished() {
        return pathfindingCommand != null && pathfindingCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathfindingCommand != null) {
            pathfindingCommand.end(interrupted);
        }
        vision.setPoseEstimationEnabled(true);
        SmartDashboard.putBoolean("DriveToPoseNew/Running", false);
        System.out.println("DriveToPoseNew: ended. Interrupted=" + interrupted);
    }
}
