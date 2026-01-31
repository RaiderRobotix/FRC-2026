package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;

public class Constants {

    //0.02208974359 PathPlanner Units per inch

    public static final COTSNeoSwerveConstants chosenModule = COTSNeoSwerveConstants.SDSMK4i(driveGearRatios.SDSMK4i_L2);
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    public static final double  kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double stickDeadband = 0.1;



    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
    
    public static final double kTrackWidth = Units.inchesToMeters(19);
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );
    public static final class ModuleConstants{
        public static final int kDrivingMotorPinionTeeth = 14;

        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0*22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    }

    public static final class Mod0 {
        
        public static final int drivermotorID0 = 10;
        public static final int anglemotorID0 = 20;
        public static final int camcoderID = 0;
        public static final double offset0 = 1.095262;

    }

    public static final class Mod1 {
        
        public static final int drivermotorID1 = 11;
        public static final int anglemotorID1 = 21;
        public static final int camcoderID1 = 1;
        public static final double offset1 = 2.543340;
    }

    public static final class Mod2 {
        
        public static final int drivermotorID2 = 12;
        public static final int anglemotorID2 = 22;
        public static final int camcoderID2 = 2;
        public static final double offset2 = -0.708699;
    }
    public static final class Mod3 {

        public static final int drivermotorID3 = 13;
        public static final int anglemotorID3 = 23;
        public static final int camcoderID3 = 3;
        public static final double offset3 = -2.411418;
    }
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}