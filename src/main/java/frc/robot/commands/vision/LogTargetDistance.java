package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

/**
* Sets the shooter power automatically based on the distance to a specific target ID using vision data.
* Note: This command is designed to run once and then finish. It does not continuously adjust 
*/
public class LogTargetDistance extends Command {
    private int targetId;
    private PhotonTrackedTarget target;
    private Vision vision;

    public LogTargetDistance(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing LogTargetDistance command");
        // this.targetId = 4; // Example target ID, change as needed
    }

    @Override
    public void execute() {
        Vision.Cameras.MAIN.updateUnreadResults();
        var results = Vision.Cameras.MAIN.getLatestResult();

        if(results.isEmpty()){
            System.out.println("Result is empty!");
            return;
        }

        var target = results.get().getBestTarget();
        
        var cameraToTarget = target.getBestCameraToTarget();
        var robotToTarget = Vision.getRobotRelativeTransformTo(target);

        double measuredDist = Math.sqrt(Math.pow(cameraToTarget.getX(), 2) + Math.pow(cameraToTarget.getY(), 2));
        double estimatedDist = vision.getDistanceFromAprilTag(targetId);
        
        SmartDashboard.putBoolean("Vision/Best Tag ID " + targetId, true);
        SmartDashboard.putString("Vision/Camera to Target Transformation", cameraToTarget.toString());
        SmartDashboard.putString("Vision/Robot to Target", robotToTarget.toString());
        SmartDashboard.putNumber("Vision/Distance to Target (VISION)", measuredDist);
        SmartDashboard.putNumber("Vision/Distance to Target (POSE ESTIMATE)", estimatedDist);
    } 


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}