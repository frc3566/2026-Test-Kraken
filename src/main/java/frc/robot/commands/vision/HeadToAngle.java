package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Snaps the robot to face forward (0° for blue alliance, 180° for red alliance
 * in blue-alliance field coordinates) while allowing full driver translation.
 *
 * Finishes as soon as the heading error drops below {@link #HEADING_TOLERANCE_DEG},
 * at which point the drivetrain's default drive command naturally takes over.
 * Pair with {@code .withTimeout()} in RobotContainer as a safety fallback.
 */
public class HeadToAngle extends Command {

    private static final double HEADING_TOLERANCE_DEG = 1.5;

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;

    private final SwerveRequest.FieldCentricFacingAngle drive;

    private Rotation2d targetHeading;

    private double degree;

    public HeadToAngle(double degree,CommandSwerveDrivetrain drivetrain,
                         DoubleSupplier velocityX, DoubleSupplier velocityY,
                         double maxSpeed, double maxAngularRate) {
        this.drivetrain = drivetrain;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.degree = degree;


        this.drive = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(maxSpeed * 0.1)
                .withRotationalDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

        drive.HeadingController.setPID(4.0, 0.0, 0.1);
        drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        boolean isBlue = DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;

        // 0° = away from blue driver station (toward red side), 180° = red alliance equivalent
        targetHeading = isBlue ? Rotation2d.fromDegrees(degree) : Rotation2d.fromDegrees(degree+180);
    }

    @Override
    public void execute() {
        drivetrain.setControl(
            drive.withVelocityX(velocityX.getAsDouble())
                 .withVelocityY(velocityY.getAsDouble())
                 .withTargetDirection(targetHeading)
        );
    }

    @Override
    public boolean isFinished() {
        double errorDeg = Math.abs(
            targetHeading.minus(drivetrain.getState().Pose.getRotation()).getDegrees()
        );
        return errorDeg < HEADING_TOLERANCE_DEG;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up — default command resumes automatically
    }
}
