package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Moves the climber to a target position (in mechanism rotations) using
 * Phoenix 6 MotionMagic closed-loop control running on the TalonFX.
 *
 * The heavy lifting (PID + motion profiling) happens inside the motor
 * controller itself — this command just sends the target and waits.
 */
public class ClimbToHeight extends Command {

    /** How close (in mechanism rotations) is "close enough" to the target. */
    private static final double TOLERANCE_ROTATIONS = 1;

    private final Climber climber;
    private final double targetRotations;
 
    /**
     * @param climber         The climber subsystem.
     * @param targetRotations Target position in mechanism rotations.
     *                        0 = fully retracted (as zeroed on startup).
     */
    public ClimbToHeight(Climber climber, double targetRotations) {
        this.climber = climber;
        this.targetRotations = targetRotations;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPosition(targetRotations);
        SmartDashboard.putNumber("Climber/TargetRotations", targetRotations);
        System.out.println("ClimbToHeight: moving to " + targetRotations + " rotations");
    }

    @Override
    public void execute() {
        // MotionMagic runs on the motor controller — nothing extra needed here.
        // Just publish telemetry so we can watch progress.
        SmartDashboard.putNumber("Climber/CurrentRotations", climber.getPosition());
        SmartDashboard.putNumber("Climber/ErrorRotations", targetRotations - climber.getPosition());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetRotations - climber.getPosition()) <= TOLERANCE_ROTATIONS;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Hold position by re-issuing the last target rather than going open-loop
            climber.setPosition(climber.getPosition());
        }
        System.out.println("ClimbToHeight: finished. Interrupted=" + interrupted
            + " | final pos=" + climber.getPosition() + " rotations");
    }
}
