package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class GetVisionData extends Command {
    private boolean targetSet = false;
    private int targetId = 0;

    public GetVisionData() {

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
            var result = results.get().getBestTarget();
            var tagID = result.getFiducialId();
            var cameraToTarget = result.getBestCameraToTarget();
            double distance = Math.sqrt(Math.pow(cameraToTarget.getX(), 2) + Math.pow(cameraToTarget.getY(), 2));
            System.out.println("Tag ID: " + tagID);
            System.out.println("Camera to Target: " + cameraToTarget);
            System.out.println("Distance: " + distance);
            if(tagID==targetId){
                targetSet = true;
            }
        } 
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Target Found!");
    }

    @Override
    public boolean isFinished() {
        return targetSet;
    }
}