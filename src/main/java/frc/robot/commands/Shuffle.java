package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Kicks the drivetrain backward then forward quickly to jostle game pieces in the hopper.
 */
public class Shuffle extends Command {
    private static final double DEFAULT_PHASE_TIME_S = 0.2;
    private static final double DEFAULT_SPEED_MPS = 2.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final double phaseTimeSec;
    private final double speedMps;
    private final Timer timer = new Timer();

    public Shuffle(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, DEFAULT_SPEED_MPS, DEFAULT_PHASE_TIME_S);
    }

    public Shuffle(CommandSwerveDrivetrain drivetrain, double speedMps, double phaseTimeSec) {
        this.drivetrain = drivetrain;
        this.speedMps = Math.abs(speedMps);
        this.phaseTimeSec = Math.max(0.05, phaseTimeSec);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        double t = timer.get();
        if (t < phaseTimeSec) {
            // First phase: drive backward quickly
            drivetrain.driveRobotRelative(new ChassisSpeeds(-speedMps, 0.0, 0.0));
        } else if (t < 2 * phaseTimeSec) {
            // Second phase: drive forward quickly
            drivetrain.driveRobotRelative(new ChassisSpeeds(speedMps, 0.0, 0.0));
        } else {
            // Hold stopped if we overrun
            drivetrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2 * phaseTimeSec);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
