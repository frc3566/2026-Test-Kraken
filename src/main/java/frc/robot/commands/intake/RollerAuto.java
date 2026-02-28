package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RollerAuto extends Command {

    private final Intake intake;
    private Timer timer;
    private double time;
    private double power;

    public RollerAuto(Intake intake, double power, double time) {
        this.intake = intake;
        this.power = power;
        this.time = time;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
        intake.rollerIn(power);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: hold position
        intake.rollerStop();
    }
}