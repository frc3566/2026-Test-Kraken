package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ArmSwitch extends Command {
    private final Intake intake;
    private boolean down;
    private Timer timer;
    private double power=0.25;
    private double time =1;


    public ArmSwitch(Intake intake, boolean down){
        this.intake = intake;
        this.down = down;
    }
    public ArmSwitch(Intake intake, boolean down, double power, double time) {
        this.intake = intake;
        this.down = down;
        this.power = power;
        this.time = time;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start();
        if(down){
            intake.armDown(power);
        } else{
            intake.armUp(power);
            time += 0.2; // More time to go up because of gravity
        }
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
        intake.stopArm();
    }
}