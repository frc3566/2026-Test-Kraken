package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {
    private final Shooter shooter;
    private final double speed;
    private final Intake intake;
    private double shootTime = 10.0; // Time to shoot after priming in seconds
    private final Timer timer = new Timer();

    public AutoShoot(Shooter shooter, Intake intake, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        this.intake = intake;
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        System.out.println("AutoShoot Command Initialized");

        timer.reset();
        timer.start();
        shooter.setLowerPower(speed);
        // shooter.setAgitatorPower(speed);
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopUpper();
        shooter.stopLower();
        intake.rollerStop();
        // shooter.stopAgitator();
        System.out.println("Prime And Shoot Command Ended" + (interrupted ? " due to interruption." : "."));
    }

    @Override
    public boolean isFinished() {
        return timer.get()>(shootTime);
    }
}
