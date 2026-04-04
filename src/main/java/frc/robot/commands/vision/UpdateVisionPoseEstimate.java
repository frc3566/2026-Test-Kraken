package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;


public class UpdateVisionPoseEstimate extends Command {
    private Vision vision;
    private CommandSwerveDrivetrain drivetrain;

    public UpdateVisionPoseEstimate(Vision vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing UpdateVisionPoseEstimate command");
        vision.updatePoseEstimation(drivetrain);
        SmartDashboard.putBoolean("Vision Pose Estimation Requested", true);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending UpdateVisionPoseEstimate command");
    //    shooter.stopUpper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}