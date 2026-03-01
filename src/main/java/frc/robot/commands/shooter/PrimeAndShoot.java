package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PrimeAndShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final double speed;
    private double primeTime = 1.0; // Default
    private double shootTime = 5.0; // Default
    private final Timer timer = new Timer();

    public PrimeAndShoot(Shooter shooter, Intake intake, double speed, double primeTime, double shootTime) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
        this.primeTime = primeTime;
        this.shootTime = shootTime;
    }

    public PrimeAndShoot(Shooter shooter, Intake intake, double speed, double primeTime) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
        this.primeTime = primeTime;
    }

    public PrimeAndShoot(Shooter shooter, Intake intake, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
    }




    @Override
    public void initialize() {
        System.out.println("Prime And Shoot Command Initialized");
        timer.reset();
        timer.start();
        // shooter.setAgitatorPower(speed);
        shooter.setUpperPower(speed);
        intake.rollerIn(0.8);
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
        intake.stopRoller();
        System.out.println("Prime And Shoot Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return timer.get()>(primeTime + shootTime);
    }
}
