package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Rotates the robot to face a specific AprilTag using a ProfiledPIDController,
 * while still allowing the driver to translate freely.
 */
public class TurnToTag extends Command {

    // Max rotational velocity: 135°/s, max rotational acceleration: 270°/s²
    private static final TrapezoidProfile.Constraints ROT_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(135),
            Units.degreesToRadians(270)
        );

    private final ProfiledPIDController rotController =
        new ProfiledPIDController(0.05, 0, 0.001, ROT_CONSTRAINTS);

    // Max number of loops to run without seeing the target
    private static final int MAX_MISSED_CYCLES = 20;

    private final CommandSwerveDrivetrain drivetrain;
    private final int tagId;
    private final DoubleSupplier velocityX; // m/s, from driver joystick
    private final DoubleSupplier velocityY; // m/s, from driver joystick

    private int missedCycles = 0;
    private boolean isFirstCycle = true;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * @param drivetrain The swerve drivetrain subsystem
     * @param tagId      The AprilTag ID to turn toward
     * @param velocityX  Supplier for forward velocity (m/s) — pass driver joystick input
     * @param velocityY  Supplier for strafe velocity (m/s) — pass driver joystick input
     */
    public TurnToTag(CommandSwerveDrivetrain drivetrain, int tagId,
                     DoubleSupplier velocityX, DoubleSupplier velocityY) {
        this.drivetrain = drivetrain;
        this.tagId = tagId;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        addRequirements(drivetrain);

        // Finish when yaw error is within 2 degrees
        rotController.setTolerance(Units.degreesToRadians(2));
    }

    @Override
    public void initialize() {
        missedCycles = 0;
        isFirstCycle = true;

        // Sample the camera immediately so we can seed the profile from the
        // robot's actual starting yaw error instead of 0.
        Vision.Cameras.MAIN.updateUnreadResults();
        var result = Vision.Cameras.MAIN.getLatestResult();

        double initialYawErrorRad = 0;
        if (!result.isEmpty()) {
            var targetOpt = result.get().getTargets().stream()
                .filter(t -> t.getFiducialId() == tagId)
                .findFirst();
            if (targetOpt.isPresent()) {
                initialYawErrorRad = Units.degreesToRadians(targetOpt.get().getYaw());
            }
        }

        // Reset from the real starting error so the trapezoid profile generates
        // a meaningful ramp from initialYawError → 0, not the trivial 0 → 0.
        rotController.reset(initialYawErrorRad);
        rotController.setGoal(0);

        SmartDashboard.putBoolean("LockToTag/Enabled", true);
        System.out.println("LockToTag enabled, targeting tag ID: " + tagId
            + " | initial yaw error: " + Units.radiansToDegrees(initialYawErrorRad) + "°");
    }

    @Override
    public void execute() {
        Vision.Cameras.MAIN.updateUnreadResults();
        var result = Vision.Cameras.MAIN.getLatestResult();

        double rotSpeed = 0;

        if (!result.isEmpty()) {
            var targetOpt = result.get().getTargets().stream()
                .filter(t -> t.getFiducialId() == tagId)
                .findFirst();

            if (targetOpt.isPresent()) {
                // getYaw() returns degrees: positive = tag is to the right
                double yawErrorRad = Units.degreesToRadians(targetOpt.get().getYaw());

                // calculate(measurement): error = goal - measurement = 0 - yawError
                // Positive yaw (tag right) → negative output → turn clockwise (right)
                rotSpeed = rotController.calculate(yawErrorRad);

                missedCycles = 0;
                SmartDashboard.putBoolean("TurnToTag/Target Found", true);
                SmartDashboard.putNumber("TurnToTag/Yaw Error (deg)", targetOpt.get().getYaw());
                SmartDashboard.putBoolean("TurnToTag/At Goal", rotController.atGoal());
            } else {
                missedCycles += isFirstCycle ? MAX_MISSED_CYCLES : 1;
                SmartDashboard.putBoolean("TurnToTag/Target Found", false);
                SmartDashboard.putNumber("TurnToTag/Missed Cycles", missedCycles);
            }
        }

        isFirstCycle = false;

        drivetrain.setControl(
            drive.withVelocityX(velocityX.getAsDouble())
                 .withVelocityY(velocityY.getAsDouble())
                 .withRotationalRate(rotSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("LockToTag/Enabled", false);
        SmartDashboard.putBoolean("LockToTag/Target Found", false);
        System.out.println("LockToTag disabled. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (missedCycles >= MAX_MISSED_CYCLES) {
            System.out.println("TurnToTag: tag not found for " + MAX_MISSED_CYCLES + " cycles, ending command.");
            return true;
        }
        return rotController.atGoal();
    }
}
