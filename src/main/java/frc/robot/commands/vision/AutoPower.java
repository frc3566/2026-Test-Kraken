package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Sets the shooter power automatically based on the distance from the robot's
 * pose estimate to a target Translation2d on the field.
 * Continuously adjusts. Tune down shooter upper kP if it oscillates too much.
 */
public class AutoPower extends Command {
    private final Shooter shooter;
    private final Supplier<Pose2d> robotPose;
    private final Supplier<Translation2d> targetTranslation;

    /**
     * @param shooter           Shooter subsystem
     * @param robotPose         Supplier of the current robot pose (e.g. drivetrain::getState().Pose)
     * @param targetTranslation Supplier of the field Translation2d to shoot at
     */
    public AutoPower(Shooter shooter, Supplier<Pose2d> robotPose, Supplier<Translation2d> targetTranslation) {
        this.shooter = shooter;
        this.robotPose = robotPose;
        this.targetTranslation = targetTranslation;
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing AutoPower command");
    }

    @Override
    public void execute() {
        Pose2d pose = robotPose.get();
        Translation2d target = targetTranslation.get();

        double distance = pose.getTranslation().getDistance(target);

        SmartDashboard.putNumber("AutoPower/Distance to Target (m)", distance);
        SmartDashboard.putString("AutoPower/Robot Translation", pose.getTranslation().toString());
        SmartDashboard.putString("AutoPower/Target Translation", target.toString());

        shooter.autoPower(distance);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutoPower command ended.");
    }

    @Override
    public boolean isFinished() {
        return false; // runs continuously until toggled off
    }
}
