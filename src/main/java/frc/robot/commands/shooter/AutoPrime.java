package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoPrime extends Command {
    private final Shooter shooter;
    private final double speed;
    private final Intake intake;
    private double primeTime; 
    private final Timer timer = new Timer();

    /**
     * Sets the upper shooter and intake roller for auto. 
     * Does not stop on its own, use with race condition.
     * 
     * @param shooter
     * @param intake
     * @param speed
     * @param primeTime
     */

    public AutoPrime(Shooter shooter, Intake intake, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
        this.addRequirements(intake);
    }

    public AutoPrime(Shooter shooter, Intake intake, double speed, double primeTime) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
        this.primeTime = primeTime;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Prime Command Initialized");
        // shooter.setAgitatorPower(speed);
        // shooter.setUpperPower(speed);
        intake.rollerIn(100);
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
