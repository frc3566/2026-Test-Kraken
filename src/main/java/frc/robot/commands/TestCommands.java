package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

public class TestCommands extends Command {

    public TestCommands() {
    }

    @Override
    public void initialize() {
        System.out.println("Test Command Initialized");
    }

    @Override
    public void execute() {
        System.out.println("Test Command Executing");
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Test Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
