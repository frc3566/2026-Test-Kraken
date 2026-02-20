package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {
    private final Shooter shooter;
    private final double speed;
    private final Timer timer;

    public AutoShoot(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        shooter.setUpperPower(speed);
        timer.reset();
        timer.start();
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
        return timer.get()>5.0; // Example: finish after 5 seconds
    }
}
