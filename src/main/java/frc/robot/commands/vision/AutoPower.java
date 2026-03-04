package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
* Sets the shooter power automatically based on the distance to a specific target ID using vision data.
* Continuously adjusts. Tune down shooter upper kP if oscillates two much
* Normally should work as long the distance don't change too rapidly
*/
public class AutoPower extends Command {
    private boolean targetSet = false;
    private int targetId;
    private Shooter shooter;
    private boolean targetFound = false;
    private PhotonTrackedTarget target;
    private Vision vision;

    public AutoPower(Shooter shooter, Vision vision, int targetId) {
        this.shooter = shooter;
        this.vision = vision;
        this.targetId = targetId;
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing AutoShoot command");
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

        var trackedTags = results.get().getTargets();

        // Check if any tags are currently tracked
        if(trackedTags.isEmpty()){
            System.out.println("No tags currently tracked!");
            return;
        }

        System.out.println("Tracked Tags: " + trackedTags.size());

        // Attempt to find the target with the specified ID among the tracked tags
        try {
            target = trackedTags.stream()
                        .filter(t -> t.getFiducialId() == targetId)
                        .findFirst()
                        .orElse(null);
        } catch (Exception e) {
            e.printStackTrace();
        }


        // If the target is found, calculate distances and set shooter power
        if(target == null){
            System.out.println("Target ID " + targetId + " not found among tracked tags.");
            return;
        }

        var cameraToTarget = target.getBestCameraToTarget();

        double measuredDist = Math.sqrt(Math.pow(cameraToTarget.getX(), 2) + Math.pow(cameraToTarget.getY(), 2));
        double estimatedDist = vision.getDistanceFromAprilTag(targetId);
        
        SmartDashboard.putBoolean("Found Target Tag ID " + targetId, true);
        SmartDashboard.putString("Camera to Target", cameraToTarget.toString());
        SmartDashboard.putNumber("Distance to Target (VISION)", measuredDist);
        SmartDashboard.putNumber("Distance to Target (POSE ESTIMATE)", estimatedDist);
        
        shooter.autoPower(measuredDist);

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