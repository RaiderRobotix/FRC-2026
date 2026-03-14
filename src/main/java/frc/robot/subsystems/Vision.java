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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;

    private final PhotonPoseEstimator photonEstimator1;
    private final PhotonPoseEstimator photonEstimator2;

    // Pose estimator that combines the vision pose estimates with the swerve
    // drive's internal pose estimation to get a more accurate estimate of the
    // robot's pose on the field.
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    // Field2d object for visualizing the robot's pose and the camera poses on the
    // field in SmartDashboard.
    private Field2d visionField = new Field2d();
    // Another debugging field for the robot's position based only on cam 1.
    private Field2d cam1Field = new Field2d();
    // Another debugging field for the robot's position based only on cam 2.
    private Field2d cam2Field = new Field2d();

    /*
     * Robot Center: C
     * Camera 1: 1
     * Camera 2: 2
     * 
     * ....(SHOOTER) FRONT (SHOOTER)
     * -y |-------------------------| +y
     * ...|...1.................2...| +x
     * ...|.........................|
     * ...|............C........... |
     * ...|.........................|
     * ...|.........................|
     * ...|-------------------------| -x
     * ...............BACK
     * 
     * The cameras are angled upwards at 60 degrees. They are also angled outwards
     * at 5 degrees.
     * 
     * The center of the robot is on the floor. A z translation of 1 unit up from the center of the robot
     * means a translation of 1 units from the floor.
     */

    // 2026 bot
    private final Transform3d kRobotToCam1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(-8.75),
                    Units.inchesToMeters(3.75)),
            new Rotation3d(0, Units.degreesToRadians(60), Units.degreesToRadians(-5)));
    private final Transform3d kRobotToCam2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(+8.75),
                    Units.inchesToMeters(3.75)),
            new Rotation3d(0, Units.degreesToRadians(60), Units.degreesToRadians(5)));

    // // Practice bot:
    // private final Transform3d kRobotToCam1 = new Transform3d(
    // new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(-8.75),
    // Units.inchesToMeters(12.5)),
    // new Rotation3d(0, Units.degreesToRadians(60), Units.degreesToRadians(-5)));
    // private final Transform3d kRobotToCam2 = new Transform3d(
    // new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(+8.75),
    // Units.inchesToMeters(12.5)),
    // new Rotation3d(0, Units.degreesToRadians(60), Units.degreesToRadians(5)));

    // The coordinates of the center of the hub on the field for each alliance.
    private final Translation2d kBlueHubPose = new Translation2d(
            fieldLayout.getTagPose(26).get().getX()
                    + Units.inchesToMeters(47.0) / 2.0,
            fieldLayout.getFieldWidth() / 2.0);

    private final Translation2d kRedHubPose = new Translation2d(
            fieldLayout.getTagPose(4).get().getX()
                    + Units.inchesToMeters(47.0) / 2.0,
            fieldLayout.getFieldWidth() / 2.0);

    // The latest standard deviations for the vision position estimates
    private Matrix<N3, N1> curStdDevs;
    // When the last valid estimate from cam1 was recieved
    private double cam1LatestResultTimestamp = Timer.getFPGATimestamp();
    // When the last valid estimate from cam2 was recieved
    private double cam2LatestResultTimestamp = Timer.getFPGATimestamp();

    // The standard deviations of our vision estimated poses, which affect
    // correction rate in SwervePoseEstimator
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.4, 0.4, 0.7);

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
        this.camera1 = new PhotonCamera(camera1);
        this.camera2 = new PhotonCamera(camera2);

        photonEstimator1 = new PhotonPoseEstimator(fieldLayout, kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(fieldLayout, kRobotToCam2);
        visionField.getObject("BlueHub").setPose(kBlueHubPose.getX(), kBlueHubPose.getY(), new Rotation2d());
        visionField.getObject("RedHub").setPose(kRedHubPose.getX(), kRedHubPose.getY(), new Rotation2d());
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

        // If there is a valid vision estimate...
        if (!visionEst.isEmpty()) {
            var est = visionEst.get();

            //Log some telemetry about each camera to SmartDashboard 
            if (camera == camera1) {
                cam1LatestResultTimestamp = est.timestampSeconds;
                cam1Field.setRobotPose(est.estimatedPose.toPose2d());
            } else if (camera == camera2) {
                cam2LatestResultTimestamp = est.timestampSeconds;
                cam2Field.setRobotPose(est.estimatedPose.toPose2d());
            }

            // Only accept measurements that are less than half a meter away from the current estimate. This helps prevent jitter from outliers.
            if (est.estimatedPose.toPose2d().getTranslation().getDistance(getPose().getTranslation()) < 0.5) {
                swervePoseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds,
                        getEstimationStdDevs());
            } else {
                // Trust measurements that are more than half a meter away less. 
                swervePoseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds,
                        kSingleTagStdDevs);
            }
        }
        return visionEst;
    }

    @Override
    public void periodic() {
        processCamera(camera1, photonEstimator1);
        processCamera(camera2, photonEstimator2);

        SmartDashboard.putNumber("Vision/Rotation", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/Distance to Hub", getDistanceFromHub());

        SmartDashboard.putBoolean("Vision/Camera 1 Connected", this.camera1.isConnected());
        SmartDashboard.putBoolean("Vision/Camera 2 Connected", this.camera2.isConnected());

        SmartDashboard.putNumber("Vision/Yaw to hub", getYawToHub());
        
        visionField.setRobotPose(getPose());
        //Log whether each camera is connected, the position of each camera on the field, and whether each camera has provided an estimate recently.
        if (this.camera1.isConnected()) {
            visionField.getObject("Camera1")
                    .setPose(getPose().plus(new Transform2d(kRobotToCam1.getTranslation().toTranslation2d(),
                            kRobotToCam1.getRotation().toRotation2d())));

            SmartDashboard.putBoolean("Camera 1 has estimate?",
                    cam1LatestResultTimestamp > Timer.getFPGATimestamp() - 1);
        } else {
            visionField.getObject("Camera1").setPose(0, 0, new Rotation2d());
        }


        if (this.camera2.isConnected()) {
            visionField.getObject("Camera2")
                    .setPose(getPose().plus(new Transform2d(kRobotToCam2.getTranslation().toTranslation2d(),
                            kRobotToCam2.getRotation().toRotation2d())));
            SmartDashboard.putBoolean("Camera 2 has estimate?",
                    cam2LatestResultTimestamp > Timer.getFPGATimestamp() - 1);
        } else {
            visionField.getObject("Camera2").setPose(0, 0, new Rotation2d());
        }

        //Log each estimate field to SmartDashboard
        SmartDashboard.putData("Vision/VisionField", visionField);
        SmartDashboard.putData("Vision/Cam1 Only Field", cam1Field);
        SmartDashboard.putData("Vision/Cam2 Only Field", cam2Field);

        SmartDashboard.putNumberArray("Vision/Component dist to hub",
                getComponentDistanceFromRobotPartToHub(new Translation2d(0, 0)));

    }

    /**
     * Returns the latest standard deviations of the estimated pose from, for use
     * with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based on number of tags, estimation strategy, and
     * distance from
     * the tags.
     * 
     * THIS IS CODE FROM PHOTONVISION'S EXAMPLE, NOT TEAM 25's.
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
     * Gets the position of the hub on the field based on the alliance color.
     * 
     * @return Position of the hub on the field as a Translation2d.
     * 
     */
    private Translation2d getHubPose() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return kRedHubPose;
        } else {
            return kBlueHubPose;
        }
    }

    /**
     * The pose is the position of the robot's center on the field. Note: the field origin is on the blue alliance wall.
     * 
     * @return the latest estimated pose of the robot on the field according to the
     *         vision system and the swerve drive pose estimator
     */
    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    /**
     * Calculates the distance from the robot to the hub using the latest estimated
     * pose of the robot and the known position of the hub on the field.
     * 
     * @return the Pythagorean distance from the center (see diagram above) of the
     *         robot to the center
     *         of the hub in meters. THIS DOES NOT INCLUDE VERTICAL (z) DISTANCE,
     *         ONLY HORIZONTAL (x,y) DISTANCE.
     */
    public double getDistanceFromHub() {
        return getPose().getTranslation().getDistance(getHubPose());
    }

    /**
     * Calculates the distance from a given robot part to the hub using the latest
     * estimated pose of the robot and the known position of the hub on the field.
     * This will return the x and y components of the distance from the robot part
     * to the hub, where the x component is the distance in the direction parallel
     * to the field's length (positive towards the opponent's side) and the y
     * component is the distance in the direction parallel to the field's width
     * (positive towards the left when looking from the driver's station).
     * 
     * @param robotPartPose The 2D (x,y) translation from the center of the robot to
     *                      the robot part in the robot's coordinate system. For
     *                      example, if the robot part is 0.5 meters in front of the
     *                      center and 0.2 meters to the left, this would be
     *                      Translation2d(0.5, 0.2).
     * @return an array containing the x and y components of the distance from the
     *         robot part to the hub in meters, where the first element is the x
     *         component and the second element is the y component.
     */
    public Double[] getComponentDistanceFromRobotPartToHub(Translation2d robotPartPose) {
        Translation2d partPositionOnField = getPose().plus(new Transform2d(robotPartPose, new Rotation2d()))
                .getTranslation();
        Translation2d diff = partPositionOnField.minus(getHubPose());
        return new Double[] { diff.getX(), diff.getY() };
    }

    /**
     * Calculates the angle from the robot to the hub using the latest estimated
     * pose of the robot and the known position of the hub on the field. The angle
     * is measured counterclockwise from the robot's forward direction.
     * 
     * @return The angle to the hub in degrees.
     */
    public double getYawToHub() {
        return getYawFromRobotPartToHub(new Translation2d(0, 0));
    }

    /**
     * Calculates the angle from a specific robot part to the hub using the latest
     * estimated
     * pose of the robot and the known position of the hub on the field. The angle
     * is measured counterclockwise from the robot's forward direction.
     * 
     * @param robotPartPose The 2D (x,y) translation from the center of the robot to
     *                      the robot part in the robot's coordinate system.
     * @return The angle to the hub in degrees.
     */
    public double getYawFromRobotPartToHub(Translation2d robotPartPose) {
        Pose2d robotPose = getPose();
        Double[] distances = getComponentDistanceFromRobotPartToHub(robotPartPose);
        double deltaX = distances[0];
        double deltaY = distances[1];
        Rotation2d angleToHub = robotPose.getRotation().minus(new Rotation2d(Math.atan2(deltaY, deltaX) + Math.PI));
        return angleToHub.getDegrees();
    }
}
