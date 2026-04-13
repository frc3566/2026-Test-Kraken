package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/**
 * Moves the intake arm to a target position (in mechanism rotations) using
 * Phoenix 6 PositionDutyCycle closed-loop control running on the TalonFX.
 *
 * The heavy lifting (PID) happens inside the motor controller — this command
 * just sends the target and waits.
 */
public class ArmToSetpoint extends Command {

    private final Intake intake;
    private final double targetRotations;
    private final double tolerance;
    private final Timer timer = new Timer();
    private double timeoutSeconds = Constants.Arm.ArmToSetpointTimeoutSeconds; // Optional timeout to prevent getting stuck if something goes wrong

    /**
     * @param intake          The intake subsystem.
     * @param targetRotations Target position in mechanism rotations.
     * @param tolerance       Acceptable error in mechanism rotations.
     */
    public ArmToSetpoint(Intake intake, double targetRotations, double tolerance, double timeoutSeconds) {
        this.intake = intake;
        this.targetRotations = targetRotations;
        this.tolerance = tolerance;
        this.timeoutSeconds = timeoutSeconds;
        addRequirements(intake);
    }

    public ArmToSetpoint(Intake intake, double targetRotations) {
        this(intake, targetRotations, Constants.Arm.DefaultToleranceRotations);
    }

    public ArmToSetpoint(Intake intake, double targetRotations, double tolerance) {
    this(intake, targetRotations, tolerance, Constants.Arm.ArmToSetpointTimeoutSeconds);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();
        intake.setArmPosition(targetRotations);
        SmartDashboard.putNumber("Intake/Arm TargetRotations", targetRotations);
        System.out.println("ArmToSetpoint: moving to " + targetRotations + " rotations");
    }

    @Override
    public void execute() {
        // PositionDutyCycle runs on the motor controller — nothing extra needed here.
        // Just publish telemetry so we can watch progress.
        SmartDashboard.putNumber("Intake/Arm CurrentRotations", intake.getArmPosition());
        SmartDashboard.putNumber("Intake/Arm ErrorRotations", targetRotations - intake.getArmPosition());
    }

    @Override
    public boolean isFinished() {
        return intake.armAtSetpoint(targetRotations, tolerance) || timer.hasElapsed(timeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopArm();
        System.out.println("ArmToSetpoint: finished. Interrupted=" + interrupted
            + " | final pos=" + intake.getArmPosition() + " rotations");
    }
}