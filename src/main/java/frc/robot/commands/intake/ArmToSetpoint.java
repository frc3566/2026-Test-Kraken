package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
public class ArmToSetpoint extends Command {
    private Intake m_Intake;
    private double setpoint; 
    private double tolerance = 0.5;

    public ArmToSetpoint(Intake m_Intake, double setpoint) {
        this.m_Intake = m_Intake;
        this.setpoint = setpoint;
        addRequirements(m_Intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("ArmToSetpoint initialized. Setpoint: " + setpoint);
        m_Intake.setArmPosition(setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentPos = m_Intake.getArmPosition();
        System.out.println("Current Position: " + currentPos + " | Setpoint: " + setpoint);

        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        double currentPos = m_Intake.getArmPosition();
        System.out.println("Current Position: " + currentPos + " | Setpoint: " + setpoint);
        System.out.println("ArmToSetpoint ended. Interrupted: " + interrupted);
        // m_Intake.stopPivot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(
            m_Intake.getArmPosition() - setpoint
        ) < tolerance;
    }
}
