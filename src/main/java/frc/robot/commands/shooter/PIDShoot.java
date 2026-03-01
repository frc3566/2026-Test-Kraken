package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PIDShoot extends Command {
    private final Shooter shooter;
    private final double speed;

    /**
     * Sets the upper shooter and intake roller for auto. 
     * Does not stop on its own, use with race condition.
     * 
     * @param shooter
     * @param intake
     * @param speed
     * @param primeTime
     */

    public PIDShoot(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("PIDShoot Command Initialized");
        // shooter.setAgitatorPower(speed);
        shooter.setUpperPower(speed);
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto Prime Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
