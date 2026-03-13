package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ArmSwitch extends Command {
    private final Intake intake;
    private final boolean down;
    private double tolerance;
    private final Timer timer = new Timer();
    private double targetRotations;


    /**
     * Moves the arm to a preset: up = 0.0 rotations, down = 0.40 rotations.
     * @param intake intake subsystem
     * @param down   true -> 0.40 rotations, false -> 0.0 rotations
     * @param tolerance acceptable error in rotations
     */
    public ArmSwitch(Intake intake, boolean down, double tolerance) {
        this.intake = intake;
        this.down = down;
        this.tolerance = tolerance;
        addRequirements(intake);
    }

    public ArmSwitch(Intake intake, boolean down) {
        this(intake, down, 0.01);
    }

    @Override
    public void initialize() {
        targetRotations = down ? 0.40 : 0.0;
        intake.setArmPosition(targetRotations);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        // Nothing else to do — TalonFX handles PositionDutyCycle on-board
    }

    @Override
    public boolean isFinished() {
        return intake.armAtSetpoint(targetRotations, tolerance) || timer.hasElapsed(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: hold position by leaving controller running; for now stop.
        intake.stopArm();
    }
}