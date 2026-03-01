package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
* Sets the shooter power automatically based on the distance to a specific target ID using vision data.
* Note: This command is designed to run once and then finish. It does not continuously adjust 
*/
public class AutoPower extends Command {
    private boolean targetSet = false;
    private int targetId;
    private Shooter shooter;
    private boolean targetFound = false;
    private PhotonTrackedTarget target;
    private Vision vision;

    public AutoPower(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing AutoShoot command");
        this.targetId = TagUtil.Side.HUB_FRONT_CENTER.getTargettingId(); // Example target ID, change as needed
    }

    @Override
    public void execute() {
        Vision.Cameras.MAIN.updateUnreadResults();
        var results = Vision.Cameras.MAIN.getLatestResult();
        
        if (!results.isEmpty()) {
            var trackedTags = results.get().getTargets();
            System.out.println("Tracked Tags: " + trackedTags.size());
            try {
                target = trackedTags.stream()
                            .filter(t -> t.getFiducialId() == targetId)
                            .findFirst()
                            .orElse(null);
            } catch (Exception e) {
                e.printStackTrace();
            }

            var cameraToTarget = target.getBestCameraToTarget();
            double measuredDist = Math.sqrt(Math.pow(cameraToTarget.getX(), 2) + Math.pow(cameraToTarget.getY(), 2));
            double estimatedDist = vision.getDistanceFromAprilTag(targetId);
            
            SmartDashboard.putBoolean("Found Target Tag ID " + targetId, true);
            SmartDashboard.putString("Camera to Target", cameraToTarget.toString());
            SmartDashboard.putNumber("Distance to Target (VISION)", measuredDist);
            SmartDashboard.putNumber("Distance to Target (POSE ESTIMATE)", estimatedDist);
            
            
            if(target!=null){
                shooter.autoPower(measuredDist);
                targetFound = true;
            } else{
                targetFound = false;
            }
            SmartDashboard.putBoolean("Target Found: " + targetId, targetFound);

        } 
        else{
            System.out.println("Result is empty!");
        }

    }

    @Override
    public void end(boolean interrupted) {
    //    shooter.stopUpper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}