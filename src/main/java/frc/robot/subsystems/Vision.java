package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private Matrix<N3, N1> curStdDevs;
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    // The transform from the robot's center to each camera. This will need to be
    // updated with the actual measurements of the robot and cameras. THIS IS IN THE
    // ROBOCENTRIC COORDINATE SYSTEM
    private final Transform3d kRobotToCam1 = new Transform3d(
            new Translation3d(0.335, -0.18, Units.inchesToMeters(13.5)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(60), Units.degreesToRadians(0)));

    private final Transform3d kRobotToCam2 = new Transform3d(
            new Translation3d(0.335, 0.18, Units.inchesToMeters(13.5)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(60), Units.degreesToRadians(0)));


    // The coordinates of the center of the hub on the field for each alliance. Don't change these.
    private final Translation2d kBlueHubPose = new Translation2d(
            fieldLayout.getTagPose(26).get().getX()
                    + Units.inchesToMeters(47.0) / 2.0,
            fieldLayout.getFieldWidth() / 2.0);

    private final Translation2d kRedHubPose = new Translation2d(
            fieldLayout.getTagPose(4).get().getX()
                    - Units.inchesToMeters(47.0) / 2.0,
            fieldLayout.getFieldWidth() / 2.0);

    private Translation2d hubPose;

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;

    private final PhotonPoseEstimator photonEstimator1;
    private final PhotonPoseEstimator photonEstimator2;

    private final SwerveDrivePoseEstimator swervePoseEstimator;

    /**
     * Constructor for the Vision subsystem. Initializes the two cameras and their
     * corresponding pose estimators
     * 
     * @param camera1     the name of the first camera as configured in the
     *                    PhotonVision software
     * @param camera2     the name of the second camera as configured in the
     *                    PhotonVision software
     * @param estConsumer the consumer that will be called with the estimated pose
     * @param alliance    the alliance the robot is on (red or blue). This is used
     *                    to
     *                    determine the location of the hub on the field, which is
     *                    necessary for calculating the distance to the hub
     */
    public Vision(Optional<Alliance> alliance, SwerveDrivePoseEstimator swervePoseEstimator, String camera1,
            String camera2) {
        this.swervePoseEstimator = swervePoseEstimator;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            hubPose = kRedHubPose;
        } else {
            hubPose = kBlueHubPose;
        }
        this.camera1 = new PhotonCamera(camera1);
        this.camera2 = new PhotonCamera(camera2);
        SmartDashboard.putBoolean("Camera 1 Connected", this.camera1.isConnected());
        SmartDashboard.putBoolean("Camera 2 Connected", this.camera2.isConnected());

        photonEstimator1 = new PhotonPoseEstimator(fieldLayout, kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(fieldLayout, kRobotToCam2);
    }

    /**
     * Processes the results from a given camera and returns an optional containing
     * the estimated robot pose if it exists
     * 
     * @param camera    the camera to process the results from
     * @param estimator the pose estimator to use to process the results from the
     *                  camera
     * @return an optional containing the estimated robot pose if it exists, or an
     *         empty optional if it doesn't
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
            updateEstimationStdDevs(visionEst, result.getTargets());
        }
        if (!visionEst.isEmpty()) {
            var est = visionEst.get();
            swervePoseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds,
                    getEstimationStdDevs());
        }
        return visionEst;
    }

    @Override
    public void periodic() {
        processCamera(camera1, photonEstimator1);
        processCamera(camera2, photonEstimator2);

        SmartDashboard.putNumber("X", getPose().getX());
        SmartDashboard.putNumber("Y", getPose().getY());
        SmartDashboard.putNumber("Rotation", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance to Hub", getDistanceFromHub());
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator1.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
    /**
     * The pose is the position of the robot's center on the field, and the rotation is the robot's orientation in the field coordinate system. 
     * @return the latest estimated pose of the robot on the field according to the vision system and the swerve drive pose estimator
     */
    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }
    /**
     * Calculates the distance from the robot to the hub using the latest estimated pose of the robot and the known position of the hub on the field. This is used for determining whether we are in range to shoot.
     * @return the Pythagorean distance from the center of the robot to the center of the hub in meters. THIS DOES NOT INCLUDE VERTICAL (z) DISTANCE, ONLY HORIZONTAL (x,y) DISTANCE.
     */
    public double getDistanceFromHub() {
        return getPose().getTranslation().getDistance(hubPose);
    }
    /**
     * Calculates the distance from a given robot part to the hub using the latest estimated pose of the robot and the known position of the hub on the field.
     * This will return the x and y components of the distance from the robot part to the hub, where the x component is the distance in the direction parallel to the field's length (positive towards the opponent's side) and the y component is the distance in the direction parallel to the field's width (positive towards the left when looking from the driver's station). 
     * 
     * @param robotPartPose The 2D (x,y) translation from the center of the robot to the robot part in the robot's coordinate system. For example, if the robot part is 0.5 meters in front of the robot's center and 0.2 meters to the left, this would be Translation2d(0.5, 0.2).
     * @return an array containing the x and y components of the distance from the robot part to the hub in meters, where the first element is the x component and the second element is the y component.
     */
    public Double[] getComponentDistanceFromRobotPartToHub(Translation2d robotPartPose) {
        Translation2d partPositionOnField = getPose().plus(new Transform2d(robotPartPose, new Rotation2d())).getTranslation();
        Translation2d diff = partPositionOnField.minus(hubPose);
        return new Double[] {diff.getX(), diff.getY()};
    }
}
