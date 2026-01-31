package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    public AHRS m_gyro;
    public final SwerveDrivePoseEstimator poseEstimator;

    // Construct the swerve modules with their respective constants.
    // The SwerveModule class will handle all the details of controlling the
    // modules.
    private final MAXSwerveModule[] swerveMods = {
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.FL),
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.FR),
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.BL),
            new MAXSwerveModule(Constants.Swerve.ModuleConstants.BR)
    };

    // Kinematics for the swerve drive, based on the module locations.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            swerveMods[0].getModuleConstants().centerOffset,
            swerveMods[1].getModuleConstants().centerOffset,
            swerveMods[2].getModuleConstants().centerOffset,
            swerveMods[3].getModuleConstants().centerOffset);
    public boolean autoOnSuccess = false;

    public Swerve() {
        // Initialize the gyro (NavX) on the USB port.
        m_gyro = new AHRS(NavXComType.kUSB1);
        // Reset the gyro to zero heading.
        m_gyro.reset();
        Timer.delay(2);
        zeroGyro();

        // Define the standard deviations for the pose estimator, which determine how
        // fast the pose estimate converges to the vision measurement. This should
        // depend on the
        // vision measurement noise and how many or how frequently vision measurements
        // are applied to the pose estimator.

        // stateStdDevs Standard deviations of the pose estimate (x position in meters,
        // y position in meters, and heading in radians). Increase these numbers to
        // trust your state estimate less
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        // Standard deviations of the vision pose measurement (x position in meters, y
        // position in meters, and heading in radians). Increase these numbers to trust
        // the vision pose measurement less.
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroYaw(), getModulePositions(), new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getEstimatedPose, // Robot pose supplier
                    this::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting
                                              // pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT RELATIVE ChassisSpeeds. Also
                                                                          // optionally outputs individual module
                                                                          // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(0.04, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
            autoOnSuccess = true;
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    public void periodic() {
        // Update the pose estimator using the wheel encoders and gyro.
        poseEstimator.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumber("Mod 0 Cancoder", swerveMods[0].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 0 Encoder", swerveMods[0].getEncoderPosition());
        SmartDashboard.putNumber("Mod 1 Cancoder", swerveMods[1].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 1 Encoder", swerveMods[1].getEncoderPosition());
        SmartDashboard.putNumber("Mod 2 Cancoder", swerveMods[2].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 2 Encoder", swerveMods[2].getEncoderPosition());
        SmartDashboard.putNumber("Mod 3 Cancoder", swerveMods[3].getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 3 Encoder", swerveMods[3].getEncoderPosition());
        SmartDashboard.putNumber("Mod 0 RotVal", swerveMods[0].getRotVal());
        SmartDashboard.putNumber("Mod 0 EncoderPosition", swerveMods[0].getEncoderPosition());
        SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());
        SmartDashboard.putBoolean("AutoOnSuccess", autoOnSuccess);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = swerveMods[0].getState();
        states[1] = swerveMods[1].getState();
        states[2] = swerveMods[2].getState();
        states[3] = swerveMods[3].getState();
        return states;
    }

    /**
     * 
     * @return The current positions of all four swerve modules as an array of
     *         SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = swerveMods[0].getPosition();
        positions[1] = swerveMods[1].getPosition();
        positions[2] = swerveMods[2].getPosition();
        positions[3] = swerveMods[3].getPosition();

        return positions;
    }

    /**
     * Zeros the gyro yaw to reset the robot's heading to zero.
     */
    public void zeroGyro() {
        m_gyro.zeroYaw();
    }

    /**
     * 
     * @return The current yaw of the robot from the gyro as a Rotation2d.
     */
    public Rotation2d getGyroYaw() {
        return m_gyro.getRotation2d();
    }

    /**
     * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
        DriverStation.reportWarning("" + visionMeasurement.getX() + ", " + visionMeasurement.getY() + ", "
                + visionMeasurement.getRotation().getDegrees(), false);
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds,
                stdDevs);
        DriverStation.reportWarning("" + visionMeasurement.getX() + ", " + visionMeasurement.getY() + ", "
                + visionMeasurement.getRotation().getDegrees(), false);
    }

    /**
     * 
     * @return The current estimated pose of the robot as a Pose2d (meters).
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose estimator to the specified pose.
     * 
     * @param pose The pose to reset the estimator to.
     */
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                pose);
    }

    /**
     * Zeros the robot's heading and resets the pose estimator to the current
     * estimated pose.
     */
    public void zeroHeading() {
        resetPoseEstimator(getEstimatedPose());
        m_gyro.reset();
    }

    /**
     * 
     * @return The current heading of the robot in degrees.
     */
    public double getHeading() {
        return getGyroYaw().getDegrees();
    }

    /**
     * 
     * @return The current robot-relative chassis speeds as a ChassisSpeeds object.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kMaxSpeedMetersPerSecond);
        swerveMods[0].setDesiredState(states[0]);
        swerveMods[1].setDesiredState(states[1]);
        swerveMods[2].setDesiredState(states[2]);
        swerveMods[3].setDesiredState(states[3]);
    }

    /**
     * Method to drive the robot using swerve kinematics.
     * 
     * @param xSpeed        The speed in the x direction (forward) as a fraction of
     *                      max speed.
     * @param ySpeed        The speed in the y direction (sideways) as a fraction of
     *                      max speed.
     * @param rot           The rotational speed as a fraction of max angular speed.
     * @param fieldRelative Whether the speeds are field-relative or robot-relative.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * Constants.Swerve.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * Constants.Swerve.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * Constants.Swerve.kMaxAngularSpeed;

        SmartDashboard.putNumber("Left Joy X Axis", xSpeedDelivered);
        SmartDashboard.putNumber("Left Joy Y Axis", ySpeedDelivered);
        SmartDashboard.putNumber("RightJoystick Right Axis", rotDelivered);

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeedDelivered,
                        ySpeedDelivered,
                        rotDelivered,
                        new Rotation2d(0))
                        : new ChassisSpeeds(
                                xSpeedDelivered,
                                ySpeedDelivered,
                                rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);

        SmartDashboard.putNumber("SwerveModState0", swerveModuleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("SwerveModState1", swerveModuleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("SwerveModState2", swerveModuleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("SwerveModState3", swerveModuleStates[3].angle.getDegrees());

        swerveMods[0].setDesiredState(swerveModuleStates[0]);
        swerveMods[1].setDesiredState(swerveModuleStates[1]);
        swerveMods[2].setDesiredState(swerveModuleStates[2]);
        swerveMods[3].setDesiredState(swerveModuleStates[3]);
    }

}
