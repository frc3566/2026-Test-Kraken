package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PrimeAndShoot extends Command {
    private final Shooter shooter;
    private final Intake intake;
    private final Supplier<Pose2d> robotPose;
    private final Supplier<Translation2d> targetTranslation;
    private double primeTime = 1.0;
    private double shootTime = 5.0;
    private final Timer timer = new Timer();

    public PrimeAndShoot(Shooter shooter, Intake intake, Supplier<Pose2d> robotPose, Supplier<Translation2d> targetTranslation, double primeTime, double shootTime) {
        this.shooter = shooter;
        this.intake = intake;
        this.robotPose = robotPose;
        this.targetTranslation = targetTranslation;
        this.primeTime = primeTime;
        this.shootTime = shootTime;
    }

    public PrimeAndShoot(Shooter shooter, Intake intake, Supplier<Pose2d> robotPose, Supplier<Translation2d> targetTranslation, double primeTime) {
        this(shooter, intake, robotPose, targetTranslation, primeTime, 5.0);
    }

    public PrimeAndShoot(Shooter shooter, Intake intake, Supplier<Pose2d> robotPose, Supplier<Translation2d> targetTranslation) {
        this(shooter, intake, robotPose, targetTranslation, 1.0, 5.0);
    }

    @Override
    public void initialize() {
        System.out.println("Prime And Shoot Command Initialized");
        timer.reset();
        timer.start();
        double distance = robotPose.get().getTranslation().getDistance(targetTranslation.get());
        shooter.autoPower(distance); // Initial spin-up
        intake.rollerIn(30);
    }

    @Override
    public void execute() {
        double distance = robotPose.get().getTranslation().getDistance(targetTranslation.get());
        shooter.autoPower(distance); // Continuously update flywheel speed as distance changes

        if (timer.get() > primeTime) {
            // Flywheel is up to speed — engage the feeder
            shooter.setLowerPower(90);
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
        return timer.get() > (primeTime + shootTime);
    }
}

