package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Swerve extends SubsystemBase{
    private final MAXSwerveModule SwerveMod0 = new MAXSwerveModule(
        Constants.Mod0.drivermotorID0,
        Constants.Mod0.anglemotorID0,
        Constants.Mod0.camcoderID,
        Constants.Mod0.offset0
    );

    private final MAXSwerveModule SwerveMod1 = new MAXSwerveModule(
        Constants.Mod1.drivermotorID1,
        Constants.Mod1.anglemotorID1,
        Constants.Mod1.camcoderID1,
        Constants.Mod1.offset1
    );

    private final MAXSwerveModule SwerveMod2 = new MAXSwerveModule(
        Constants.Mod2.drivermotorID2,
        Constants.Mod2.anglemotorID2,
        Constants.Mod2.camcoderID2,
        Constants.Mod2.offset2
    );

    private final MAXSwerveModule SwerveMod3 = new MAXSwerveModule(
        Constants.Mod3.drivermotorID3,
        Constants.Mod3.anglemotorID3,
        Constants.Mod3.camcoderID3,
        Constants.Mod3.offset3
    );

   




    public AHRS m_gyro;

    SwerveDriveOdometry m_Odometry;
    
    public boolean autonsuccess = false;

    public Swerve(){
        m_gyro = new AHRS(NavXComType.kUSB1);
        m_gyro.reset();
        Timer.delay(2);
        zeroGyro();

        
        m_Odometry = new SwerveDriveOdometry(
            Constants.kDriveKinematics, 
            getYaw(),
            new SwerveModulePosition[] {
                SwerveMod0.getPosition(),
                SwerveMod1.getPosition(),
                SwerveMod2.getPosition(),
                SwerveMod3.getPosition()
            }
        );

        RobotConfig config;
        try{
        config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(0.04, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
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
        autonsuccess = true;
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }  
        
    }

    @Override
    public void periodic(){
        m_Odometry.update(
            getYaw(),
            new SwerveModulePosition[] {
                SwerveMod0.getPosition(),
                SwerveMod1.getPosition(),
                SwerveMod2.getPosition(),
                SwerveMod3.getPosition()
            }

        );
        SmartDashboard.putNumber("Mod 0 Cancoder", SwerveMod0.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 0 Encoder", SwerveMod0.getEncoderPosition());
        SmartDashboard.putNumber("Mod 1 Cancoder", SwerveMod1.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 1 Encoder", SwerveMod1.getEncoderPosition());
        SmartDashboard.putNumber("Mod 2 Cancoder", SwerveMod2.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 2 Encoder", SwerveMod2.getEncoderPosition());
        SmartDashboard.putNumber("Mod 3 Cancoder", SwerveMod3.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod 3 Encoder", SwerveMod3.getEncoderPosition());
        SmartDashboard.putNumber("Mod 0 RotVal", SwerveMod0.getRotVal());
        SmartDashboard.putNumber("Mod 0 EncoderPosition", SwerveMod0.getEncoderPosition());
        SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
        SmartDashboard.putBoolean("AutonSuccess", autonsuccess);
        
    }

    public void zeroGyro(){
        m_gyro.zeroYaw();
        
    }

    public void resetPose(Pose2d pose) {
        m_Odometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getYaw(){
        return (false) ? Rotation2d.fromDegrees(360 - m_gyro.getFusedHeading()) : Rotation2d.fromDegrees(m_gyro.getFusedHeading());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kMaxSpeedMetersPerSecond);
        SwerveMod0.setDesiredState(states[0]);
        SwerveMod1.setDesiredState(states[1]);
        SwerveMod2.setDesiredState(states[2]);
        SwerveMod3.setDesiredState(states[3]);
    }

    public Pose2d getPose(){
        return m_Odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        m_Odometry.resetPosition(
            getYaw(),
            new SwerveModulePosition[] {
                SwerveMod0.getPosition(),
                SwerveMod1.getPosition(),
                SwerveMod2.getPosition(),
                SwerveMod3.getPosition()
            },
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        double xSpeedDelivered = xSpeed * Constants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * Constants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * Constants.kMaxAngularSpeed;
        
        SmartDashboard.putNumber("Left Joy X Axis", xSpeedDelivered);
        SmartDashboard.putNumber("Left Joy Y Axis", ySpeedDelivered);
        SmartDashboard.putNumber("RightJoystick Right Axis", rotDelivered);

        SwerveModuleState[] swerveModuleStates =
            Constants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    new Rotation2d(0.0)
                )
                : new ChassisSpeeds(
                        xSpeedDelivered,
                        ySpeedDelivered,
                        rotDelivered
                    )
                );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
    
    SmartDashboard.putNumber("SwerveModState0", swerveModuleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("SwerveModState1", swerveModuleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("SwerveModState2", swerveModuleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("SwerveModState3", swerveModuleStates[3].angle.getDegrees());

    SwerveMod0.setDesiredState(swerveModuleStates[0]);
    SwerveMod1.setDesiredState(swerveModuleStates[1]);
    SwerveMod2.setDesiredState(swerveModuleStates[2]);
    SwerveMod3.setDesiredState(swerveModuleStates[3]);



        /*
        var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative  
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, 
                    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
        SwerveMod0.setDesiredState(swerveModuleStates[0]);
        SwerveMod1.setDesiredState(swerveModuleStates[1]);
        SwerveMod2.setDesiredState(swerveModuleStates[2]);
        SwerveMod3.setDesiredState(swerveModuleStates[3]);
        */

        //System.out.println(swerveModuleStates[0].angle.getDegrees());
    }

    public void zeroHeading() {
        m_Odometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = SwerveMod0.getState();
        states[1] = SwerveMod1.getState();
        states[2] = SwerveMod2.getState();
        states[3] = SwerveMod3.getState();
        return states;
    }

    public Rotation2d getGyroYaw() {
        return (Constants.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getFusedHeading()) : Rotation2d.fromDegrees(m_gyro.getFusedHeading());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = SwerveMod0.getPosition();
        positions[1] = SwerveMod1.getPosition();
        positions[2] = SwerveMod2.getPosition();
        positions[3] = SwerveMod3.getPosition();

        return positions;
    }

    public double getHeading() {
        return getYaw().getDegrees();
    }
    

}
