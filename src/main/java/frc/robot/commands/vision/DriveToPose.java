package frc.robot.commands.vision;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Drives the robot to a target {@link Pose2d} (position + heading) using:
 * - Separate X and Y {@link ProfiledPIDController}s for translation
 * - {@link SwerveRequest.FieldCentricFacingAngle} for heading (built-in HeadingController)
 *
 * All three axes converge simultaneously. The command ends when both
 * position and heading are within tolerance.
 *
 * Example: robot at (0,0), call with Pose2d(3, 2, Rotation2d.fromDegrees(180))
 * → robot drives to (3,2) and faces back toward (2,2).
 */
public class DriveToPose extends Command {

    // TODO: Tune these gains on the real robot
    private static final double kP_TRANSLATION              = 6.0;
    private static final double kD_TRANSLATION              = 0.0;
    private static final double MAX_SPEED_MPS               = 3.79;  // m/s
    private static final double MAX_ACCEL_MPS2              = 3.0;  // m/s²
    private static final double POSITION_TOLERANCE_METERS   = 0.1; // 10 cm
    private static final double HEADING_TOLERANCE_DEG       = 2.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;

    // Separate controllers for X and Y so each axis is independently profiled
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;

    // FieldCentricFacingAngle runs the heading PID inside the swerve request itself.
    // ForwardPerspective MUST be BlueAlliance so that withVelocityX/Y are always in the
    // absolute field frame (matching odometry). OperatorPerspective (the default) would
    // rotate these axes by whatever seedFieldCentric() last set, causing wrong positions.
    private final SwerveRequest.FieldCentricFacingAngle drive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withMaxAbsRotationalRate(RotationsPerSecond.of(0.75).in(RadiansPerSecond))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); // field-relative, not robot-relative


    /**
     * @param drivetrain         The swerve drivetrain subsystem.
     * @param targetPoseSupplier Supplier for the target {@link Pose2d}.
     *                           Evaluated once in {@link #initialize()}, so it can
     *                           be dynamic (e.g. computed from a tag pose + offset).
     */
    public DriveToPose(CommandSwerveDrivetrain drivetrain, Vision vision, Supplier<Pose2d> targetPoseSupplier) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetPoseSupplier = targetPoseSupplier;

        var translationConstraints = new TrapezoidProfile.Constraints(MAX_SPEED_MPS, MAX_ACCEL_MPS2);
        xController = new ProfiledPIDController(kP_TRANSLATION, 0, kD_TRANSLATION, translationConstraints);
        yController = new ProfiledPIDController(kP_TRANSLATION, 0, kD_TRANSLATION, translationConstraints);

        xController.setTolerance(POSITION_TOLERANCE_METERS);
        yController.setTolerance(POSITION_TOLERANCE_METERS);

        // TODO: Tune via SysId rotation routine in CommandSwerveDrivetrain
        drive.HeadingController.setPID(3.0, 0, 0.1);
        drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        vision.setPoseEstimationEnabled(false); // Prevent vision from fighting the command's control over the drivetrain
        targetPose = targetPoseSupplier.get();
        Pose2d currentPose = drivetrain.getPose();

        // Reset profiles from the robot's ACTUAL current position AND velocity so the
        // trapezoid ramps from here → target without a jerk if the robot is already moving.
        // getState().Speeds is robot-relative; rotate it to the field frame using current heading.
        var robotSpeeds = drivetrain.getState().Speeds;
        double headingRad = currentPose.getRotation().getRadians();
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());

        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());

        SmartDashboard.putString("DriveToPose/Target Pose", targetPose.toString());
        System.out.println("DriveToPose: driving to " + targetPose);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double vx = xController.atGoal() ? 0.0 : xController.calculate(currentPose.getX());
        double vy = yController.atGoal() ? 0.0 : yController.calculate(currentPose.getY());

        SmartDashboard.putBoolean("DriveToTrench/Running", true);
        SmartDashboard.putNumber("DriveToPose/X Error (m)", targetPose.getX() - currentPose.getX());
        SmartDashboard.putNumber("DriveToPose/Y Error (m)", targetPose.getY() - currentPose.getY());
        SmartDashboard.putNumber("DriveToPose/Heading Error (deg)",
            targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        SmartDashboard.putBoolean("DriveToPose/At Goal", atGoal());

        drivetrain.setControl(
            drive.withVelocityX(vx)
                 .withVelocityY(vy)
                 .withTargetDirection(targetPose.getRotation())
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion cleanly
        drivetrain.setControl(new SwerveRequest.Idle());
        vision.setPoseEstimationEnabled(true);
        SmartDashboard.putBoolean("DriveToTrench/Running", false);
        System.out.println("DriveToPose: ended. Interrupted=" + interrupted);
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    /**
     * Returns true when the robot is within position and heading tolerances.
     * Can be polled externally (e.g. from a sequential command group).
     */
    public boolean atGoal() {
        boolean headingOnTarget = Math.abs(
            targetPose.getRotation().minus(drivetrain.getPose().getRotation()).getDegrees()
        ) < HEADING_TOLERANCE_DEG;
        return xController.atGoal() && yController.atGoal() && headingOnTarget;
    }
}
