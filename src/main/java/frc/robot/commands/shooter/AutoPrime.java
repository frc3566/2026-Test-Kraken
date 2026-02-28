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

    public AutoPrime(Shooter shooter, Intake intake, double speed, double primeTime) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
        this.primeTime = primeTime;
    }

    @Override
    public void initialize() {
        System.out.println("Prime Command Initialized");
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
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // shooter.stopAgitator();
        System.out.println("Prime And Shoot Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return timer.get()>(primeTime);
    }
}
