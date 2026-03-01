package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    // The transform from the robot's center to each camera. This will need to be updated with the actual measurements of the robot and cameras
    private final Transform3d kRobotToCam1 = new Transform3d();
    private final Transform3d kRobotToCam2 = new Transform3d();
    // The coordinates of the center of the hub on the field for each alliance
    private final Translation3d redHubPose = new Translation3d(Units.inchesToMeters(651.22-182.11), Units.inchesToMeters(317.69/2), 0);
    private final Translation3d blueHubPose = new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(317.69/2), 0);

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private Translation3d hubPose;

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;

    private final PhotonPoseEstimator photonEstimator1;
    private final PhotonPoseEstimator photonEstimator2;

    private double[] distanceToHubArray = new double[2];
    private double[] yawArray = new double[2];
    private double[] pitchArray = new double[2];
    private double[] rollArray = new double[2];

    /**
     * Constructor for the Vision subsystem. Initializes the two cameras and their corresponding pose estimators
     * @param camera1 the name of the first camera as configured in the PhotonVision software
     * @param camera2 the name of the second camera as configured in the PhotonVision software
     * @param alliance the alliance the robot is on (red or blue). This is used to determine the location of the hub on the field, which is necessary for calculating the distance to the hub
     */
    public Vision(String camera1, String camera2, Optional<Alliance> alliance) {
        if(alliance.isPresent() && alliance.get() == Alliance.Red) {
            hubPose = redHubPose;
        } else {
            hubPose = blueHubPose;
        }

        this.camera1 = new PhotonCamera(camera1);
        this.camera2 = new PhotonCamera(camera2);

        photonEstimator1 = new PhotonPoseEstimator(fieldLayout, kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(fieldLayout, kRobotToCam2);
    }

    /**
     * Processes the results from a given camera and returns an optional containing the estimated robot pose if it exists
     * @param camera the camera to process the results from
     * @param estimator the pose estimator to use to process the results from the camera
     * @return an optional containing the estimated robot pose if it exists, or an empty optional if it doesn't
     */
    private Optional<EstimatedRobotPose> processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        // Process each new result from the camera
        for (var result : camera.getAllUnreadResults()) {
            // Try this more accurate position estimator...
            visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                // ...and try this if it fails
                visionEst = estimator.estimateLowestAmbiguityPose(result);
            }
        }
        return visionEst;
    }

    @Override
    public void periodic() {
        processCamera(camera1, photonEstimator1).ifPresent(
                est -> {
                    updateState(est, 1);
                });
        processCamera(camera2, photonEstimator2).ifPresent(
                est -> {
                    updateState(est, 2);
                });
        outputToSmartDashboard();
    }
    /**
     * Updates the yaw, pitch, roll, and distance to the hub arrays with the latest information from a given camera
     * @param est the estimated robot pose from the camera
     * @param cameraNum which camera the information is from (1 or 2)
     */
    private void updateState(EstimatedRobotPose est, int cameraNum) {
        yawArray[cameraNum - 1] = est.estimatedPose.getRotation().getZ();
        pitchArray[cameraNum - 1] = est.estimatedPose.getRotation().getY();
        rollArray[cameraNum - 1] = est.estimatedPose.getRotation().getX();
        distanceToHubArray[cameraNum - 1] = getDistanceToPose(est, hubPose);
    }
    /** 
     * Outputs the yaw, pitch, roll, and distance to the hub to smartDashboard for debugging purposes
    */
    private void outputToSmartDashboard() {
        SmartDashboard.putNumber("Vision Yaw", getYawVal());
        SmartDashboard.putNumber("Vision Pitch", getPitchVal());
        SmartDashboard.putNumber("Vision Roll", getRollVal());
        SmartDashboard.putNumber("Vision Distance", getDistanceFromHub());
    }

    /**
     * Returns the distance from the robot to a given pose
     * 
     * @param robotPose the pose of the robot
     * @param otherPose the pose to calculate the distance to
     * @return the distance to the given pose in meters
     */
    public double getDistanceToPose(EstimatedRobotPose robotPose, Translation3d otherPose) {
        return robotPose.estimatedPose.getTranslation().getDistance(otherPose);
    }

    /**
     * Returns the yaw of the robot by averaging the yaw as calculated by each camera 
     * @return the yaw in degrees
     */
    public double getYawVal() {
        return (yawArray[0] + yawArray[1]) / 2;
    }

    /**
     * Returns the pitch of the robot by averaging the pitch as calculated by each camera 
     * @return the pitch in degrees
     */
    public double getPitchVal() {
        return (pitchArray[0] + pitchArray[1]) / 2;
    }

    /**
     * Returns the roll of the robot by averaging the roll as calculated by each camera 
     * @return the roll in degrees
     */
    public double getRollVal() {
        return (rollArray[0] + rollArray[1]) / 2;
    }

    /**
     * Returns the distance to the center of the hub by averaging the distance to
     * the hub as calculated by each cameras
     * 
     * @return the distance to the center of the hub in meters
     */
    public double getDistanceFromHub() {
        return (distanceToHubArray[0] + distanceToHubArray[1]) / 2;
    }
}
