package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ToggleFire extends Command {
    private final Shooter shooter;
    private final double speed;

    public ToggleFire(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("Toggle Fire Command Initialized");
        shooter.setUpperPower(speed);
    }

    @Override
    public void execute() {
        
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Toggle Fire Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
