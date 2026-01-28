package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class GetVisionData extends Command {
    private boolean targetSet = false;
    private int tagID = 0;

    public GetVisionData() {

    }

    @Override
    public void initialize() {
        targetSet = false;
        tagID = 0;
    }

    @Override
    public void execute() {
        // Shuffleboard.getTab("Vision").addNumber("Tag ID", () -> tagID);
        // Shuffleboard.getTab("Vision").addBoolean("Has Target", () -> targetSet);
        Vision.Cameras.MAIN.updateUnreadResults();
        var results = Vision.Cameras.MAIN.getLatestResult();
        
        if (!results.isEmpty()) {
            targetSet = true;
            var result = results.get().getBestTarget();
            tagID = result.getFiducialId();
            System.out.println(result.getBestCameraToTarget());

            
        } else {
            targetSet = false;
            tagID = -1;
        }

        System.out.println(String.valueOf(tagID) + String.valueOf(targetSet));
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