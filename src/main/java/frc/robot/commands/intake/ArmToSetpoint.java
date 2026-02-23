package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ArmToSetpoint extends Command {

    private final Intake intake;
    private final double targetRotations;
    private final double tolerance; // rotations (~7 degrees)

    public ArmToSetpoint(Intake intake, double targetRotations, double tolerance) {
        this.intake = intake;
        this.targetRotations = targetRotations;
        this.tolerance = tolerance;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Current Arm Pos:" + intake.getArmPosition());
        System.out.println("Initiating ArmToSetpoint...");
        intake.setArmPosition(targetRotations);
    }

    @Override
    public void execute(){
        System.out.println("[PID IN ACTION] Current Arm Pos:" + intake.getArmPosition());

    }

    @Override
    public boolean isFinished() {
        return intake.armAtSetpoint(targetRotations, tolerance);
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: hold position
        System.out.println("ArmToSetpoint Ended. Arm Pos:" + intake.getArmPosition());
    }
}