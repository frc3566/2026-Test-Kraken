package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PrimeAndShoot extends Command {
    private final Shooter shooter;
    private final double speed;
    private double primeTime = 1.0; // Time to prime the shooter in seconds
    private double shootTime = 1.0; // Time to shoot after priming in seconds
    private final Timer timer = new Timer();

    public PrimeAndShoot(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("Prime And Shoot Command Initialized");
        timer.reset();
        timer.start();
        shooter.setUpperPower(speed);
    }

    @Override
    public void execute() {
        if (timer.get()>primeTime) { // Adjust the time as needed
            System.out.println("Prime And Shoot Command: Lower shooter primed, starting upper shooter.");
            shooter.setLowerPower(speed);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopUpper();
        shooter.stopLower();
        System.out.println("Prime And Shoot Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return timer.get()>(primeTime + shootTime);
    }
}
