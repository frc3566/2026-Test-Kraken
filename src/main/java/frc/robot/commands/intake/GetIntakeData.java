package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class GetIntakeData extends Command {
    private double rotation;
    private Intake intake;

    public GetIntakeData(Intake intake) {
        this.intake = intake;
        this.rotation = intake.getArmRotation();
    }

    @Override
    public void initialize() {
        System.out.println(rotation);
    }

    @Override
    public void execute() {
        // Shuffleboard.getTab("Vision").addNumber("Tag ID", () -> tagID);
        // Shuffleboard.getTab("Vision").addBoolean("Has Target", () -> targetSet);
        rotation = intake.getArmRotation();
        SmartDashboard.putNumber("Current Arm Rotation", rotation);
        // System.out.println(String.valueOf(tagID) + String.valueOf(targetSet));
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("Target Found!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}