package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoPrime extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final Timer timer = new Timer();
    private int timeOut = -1;

    /**
     * Sets the upper shooter and intake roller for auto. 
     * Does not stop on its own, use with race condition.
     * 
     * @param shooter
     * @param intake
     * @param speed
     * @param primeTime
     */
    public AutoPrime(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    public AutoPrime(Shooter shooter, Intake intake, int timeOut) {
        this(shooter, intake);
        this.timeOut = timeOut;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("Prime Command Initialized");
        // shooter.setAgitatorPower(speed);
        // shooter.setUpperPower(speed);
        intake.rollerIn(30);
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
        if(timeOut != -1 && timer.hasElapsed(timeOut)) {
            return true;
        }   

        return false;
    }
}
