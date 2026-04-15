package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Prime and shoot with a hardcoded RPS value.
 * Use this for testing/fallback when pose-based AutoPower is not available or trusted.
 */
public class PrimeAndShootFixed extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final double speed; // In rotations per sec
    private double primeTime = 1.0;
    private double shootTime = 5.0;
    private final Timer timer = new Timer();

    public PrimeAndShootFixed(Shooter shooter, Intake intake, double rps, double primeTime, double shootTime) {
        this.shooter = shooter;
        this.speed = rps;
        this.intake = intake;
        this.primeTime = primeTime;
        this.shootTime = shootTime;
        addRequirements(shooter, intake);
    }

    public PrimeAndShootFixed(Shooter shooter, Intake intake, double speed, double primeTime) {
        this(shooter, intake, speed, primeTime, 5.0);
    }

    public PrimeAndShootFixed(Shooter shooter, Intake intake, double speed) {
        this(shooter, intake, speed, 1.0, 5.0);
    }

    @Override
    public void initialize() {
        System.out.println("PrimeAndShootFixed Command Initialized at " + speed + " RPS");
        timer.reset();
        timer.start();
        shooter.setUpperPower(speed);
        // intake.rollerIn(30);
    }

    @Override
    public void execute() {
        if (timer.get() > primeTime) {
            shooter.setLowerPower(90);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopUpper();
        shooter.stopLower();
        intake.stopRoller();
        System.out.println("PrimeAndShootFixed Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return timer.get() > (primeTime + shootTime);
    }
}
