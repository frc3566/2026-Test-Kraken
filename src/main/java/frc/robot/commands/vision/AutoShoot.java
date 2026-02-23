package frc.robot.commands.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


public class AutoShoot extends Command {
    private boolean targetSet = false;
    private int targetId = 4;
    private Shooter shooter;
    private boolean targetFound = false;
    private PhotonTrackedTarget target;

    public AutoShoot(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing GetVisionData command");
    }

    @Override
    public void execute() {
        Vision.Cameras.MAIN.updateUnreadResults();
        var results = Vision.Cameras.MAIN.getLatestResult();
        
        if (!results.isEmpty()) {
            var trackedTags = results.get().getTargets();
            try {
                target = trackedTags.stream()
                            .filter(t -> t.getFiducialId() == targetId)
                            .findFirst()
                            .orElse(null);
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }

            var cameraToTarget = target.getBestCameraToTarget();
            double distance = Math.sqrt(Math.pow(cameraToTarget.getX(), 2) + Math.pow(cameraToTarget.getY(), 2));
            // System.out.println("Tag ID: " + tagID);
            System.out.println("Camera to Target: " + cameraToTarget);
            System.out.println("Distance: " + distance);
            if(target!=null){
                shooter.autoPower(distance);
                targetFound = true;
            }
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