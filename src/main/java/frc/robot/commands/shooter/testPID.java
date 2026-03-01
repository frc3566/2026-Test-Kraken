package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class testPID extends Command {
    private final Shooter shooter;
    private double currSpeed; // In RPM
    private  double goalSpeed;
    private final Intake intake;
    private double shootTime = 5.0; // Time to shoot after priming in seconds
    private final Timer timer = new Timer();

    public testPID(Shooter shooter, Intake intake, double goalSpeed) {
        this.shooter = shooter;
        this.goalSpeed = goalSpeed;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        System.out.println("TestPID Command Initialized");
        timer.reset();
        timer.start();
        

        // shooter.setAgitatorPower(speed);
    }

    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopUpper();
        // shooter.stopAgitator();
        System.out.println("TestPID Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(shootTime);
    }
}
