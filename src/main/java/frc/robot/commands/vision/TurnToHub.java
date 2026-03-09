package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Continuously rotates the robot to face a specific AprilTag using
 * {@link SwerveRequest.FieldCentricFacingAngle}, while still allowing the
 * driver to translate freely.
 *
 * The target heading is computed from the robot's odometry pose and the tag's
 * known field position every execute() cycle, so it stays accurate as the
 * robot moves. No camera needed — relies on pose estimation being accurate.
 */
public class TurnToHub extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier velocityX; // m/s, field-relative
    private final DoubleSupplier velocityY; // m/s, field-relative
    private boolean isBlueAlliance; // resolved in initialize(), not at construction time

    /**
     * FieldCentricFacingAngle handles its own heading PID (HeadingController)
     * internally on the swerve request. We just set the target Rotation2d.
     *
     * TODO: Tune HeadingController kP/kD via Phoenix Tuner X SysId rotation routine.
     *       Starting values below are conservative — increase kP if slow to snap.
     */
    private final SwerveRequest.FieldCentricFacingAngle drive;

    /** Set to true in initialize() if the tag ID is not in the field layout. */
    private boolean tagNotInLayout = false;

    /** Cached tag translation — doesn't change after initialize(). */
    private Translation2d center;

    /**
     * @param drivetrain The swerve drivetrain subsystem.
     * @param tagId      The AprilTag ID to continuously face.
     * @param velocityX  Forward velocity supplier (m/s) from driver joystick.
     * @param velocityY  Strafe velocity supplier (m/s) from driver joystick.
     */
    public TurnToHub(CommandSwerveDrivetrain drivetrain,
                        DoubleSupplier velocityX, DoubleSupplier velocityY, double MaxSpeed, double MaxAngularRate) {
        this.drivetrain = drivetrain;
        this.velocityX = velocityX;
        this.velocityY = velocityY;

        this.drive =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(0) // Heading PID manages rotation; deadband would cause steady-state error
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective); // field-relative, not robot-relative


        addRequirements(drivetrain);

        // P: raised for faster snap. I: eliminates the steady-state 2° error that P alone can't correct.
        // D: increased slightly to dampen the higher P. iZone: only integrate when close to target
        // to prevent windup during large initial turns.
        drive.HeadingController.setPID(4.0, 0.0, 0.1);
        drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Read alliance here, not at construction time — DS alliance is not available until
        // the robot is enabled and the DS has communicated the alliance color.
        isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        center = TagUtil.getHubCenterTranslation();

        SmartDashboard.putBoolean("TurnToHub/Enabled", true);
    }

    @Override
    public void execute() {

        var robotPose = drivetrain.getPose();

        // Compute the field-relative angle from the robot to the tag
        double dx = center.getX() - robotPose.getX();
        double dy = center.getY() - robotPose.getY();
        Rotation2d targetHeading = new Rotation2d(Math.atan2(dy, dx));

        // Add 180 degrees to adjust for field oriented (red front = 180 deg)
        if(!isBlueAlliance){
            targetHeading = targetHeading.plus(Rotation2d.kPi);
            // System.out.println("TurnToHub: Red alliance - adding 180 degrees to target heading");
        } else{
            // System.out.println("TurnToHub: Blue alliance - using target heading as is");

        }

        double headingError = targetHeading.minus(robotPose.getRotation()).getDegrees();

        SmartDashboard.putNumber("TurnToHub/Target Heading (deg)", targetHeading.getDegrees());
        SmartDashboard.putNumber("TurnToHub/Heading Error (deg)", headingError);
        SmartDashboard.putNumber("TurnToHub/Distance to Tag (m)",
            robotPose.getTranslation().getDistance(center));

        drivetrain.setControl(
            drive.withVelocityX(velocityX.getAsDouble())
                 .withVelocityY(velocityY.getAsDouble())
                 .withTargetDirection(targetHeading)
        );
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("TurnToHub/Enabled", false);
        SmartDashboard.putNumber("TurnToHub/Target Heading (deg)", 0);
        SmartDashboard.putNumber("TurnToHub/Heading Error (deg)", 0);
        System.out.println("TurnToHub: ended. Interrupted=" + interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}