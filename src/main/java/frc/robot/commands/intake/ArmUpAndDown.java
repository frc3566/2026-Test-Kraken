package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Jiggles the intake arm back and forth between two setpoints.
 * Default setpoints: 0.40 (down-ish) and 0.20 (up-ish), switching every 0.2s.
 */
public class ArmUpAndDown extends Command {
    private final Intake intake;
    private final Timer timer = new Timer();
    private final double highRotations;
    private final double lowRotations;
    private final double intervalSeconds;

    private double currentTarget;

    /**
     * @param intake intake subsystem
     * @param intervalSeconds time between toggles
     * @param highRotations upper setpoint (rotations)
     * @param lowRotations lower setpoint (rotations)
     */
    public ArmUpAndDown(Intake intake, double intervalSeconds, double highRotations, double lowRotations) {
        this.intake = intake;
        this.intervalSeconds = intervalSeconds;
        this.highRotations = highRotations;
        this.lowRotations = lowRotations;
        addRequirements(intake);
    }

    /**
     * Uses default values: interval 0.5s, high 0.40, low 0.30 rotations.
     */
    public ArmUpAndDown(Intake intake) {
        this(intake, 0.35, 0.40, 0.10);
    }


    @Override
    public void initialize() {
        currentTarget = highRotations;
        intake.setArmPosition(currentTarget);
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(intervalSeconds)) {
            currentTarget = (currentTarget == highRotations) ? lowRotations : highRotations;
            intake.setArmPosition(currentTarget);
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted (e.g., button released)
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopArm();
    }
}
