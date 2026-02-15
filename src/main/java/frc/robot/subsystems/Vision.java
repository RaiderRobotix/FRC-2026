package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase{
    //Our camera we will be using for vision purposes
    private final PhotonCamera camera;

    //The results given by the camera
    private PhotonPipelineResult result;
    
    //The list of targets based off the results
    private List<PhotonTrackedTarget> targets;

    //Yaw of the target
    private double yawval = 0.0;

    //Pitch of the target
    private double pitchval = 0.0;
    
    //Creates the vision object from the name passed
    public Vision(String name){
        camera = new PhotonCamera(name);
    }

    //Periodically updates the result, the target list, the yaw, and the pitch, and outputs information to smartDashboard
    @Override
    public void periodic(){
        result = camera.getLatestResult();
        targets = result.getTargets();
        yawval = updateTargetYaw(targets, checkForTarget(targets));
        pitchval = updateTargetPitch(targets, checkForTarget(targets));
        SmartDashboard.putNumber("AprilTagYawVal", yawval);
        SmartDashboard.putNumber("AprilTagPitch", pitchval);
        SmartDashboard.putNumber("AprilTagDistanceFromPitch", getDistanceFromPitchVal());
    }

    //Goes through the list of targets and checks for a specific apriltag id
    public int checkForTarget(List<PhotonTrackedTarget> targets){
        for(int i = 0; i < targets.size(); i++){
            if(targets.get(i).getFiducialId() == 3){
                return i;
            }
        }
        //Returns -1 if it can't find any
        return -1;
    }

    //Updates the yaw value
    public double updateTargetYaw(List<PhotonTrackedTarget> targets, int id){
        if(id == -1){
            return 5;
        }
        double num = targets.get(0).getYaw();
        return num;
    }

    //Updates the pitch value
    public double updateTargetPitch(List<PhotonTrackedTarget> targets, int id){
        if(id == -1){
            return 0.0;
        }
        double num = targets.get(0).getPitch();
        return num;
    }

    //Returns the yaw
    public double getYawVal(){
        return yawval;
    }

    //Returns the distance from the pitch value using a lot of trignometry [very hard work :(]
    public double getDistanceFromPitchVal(){
        return (51.0-12.0)/Math.tan((Math.PI*(pitchval+30))/180);
    }

}
