package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;

public class Constants {

    //0.0238 PathPlanner Units per inch

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

        public static final double kDrivingMotorReduction = (45.0*16.0*50.0) / (kDrivingMotorPinionTeeth * 15.0 * 28.0);
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
    public static final class AprilTagLocations {
        //Blue Features
        public static final int blueCenterOutpostMiddle = 26;
        public static final int blueCenterOutpostLeft = 25;
        public static final int blueLeftOuterTrench = 23;
        public static final int blueLeftInnerTrench = 22;
        public static final int blueRightOuterTrench = 28;
        public static final int blueRightInnerTrench = 17;
        public static final int blueLeftOutpostMiddle = 21;
        public static final int blueLeftOutpostRight = 24;
        public static final int blueRightOutpostMiddle = 18;
        public static final int blueRightOutpostLeft = 27;
        public static final int blueRearOutpostLeft = 19;
        public static final int blueRearOutpostRight = 20;
        public static final int blueHumanPlayerLeft = 29;
        public static final int blueHumanPlayerRight = 30;
        public static final int blueClimbLeft = 31;
        public static final int blueClimbRight = 32;
        
        //Red Features
        public static final int redCenterOutpostMiddle = 10;
        public static final int redCenterOutpostLeft = 9;
        public static final int redLeftOuterTrench = 7;
        public static final int redLeftInnerTrench = 6;
        public static final int redRightOuterTrench = 12;
        public static final int redRightInnerTrench = 1;
        public static final int redLeftOutpostMiddle = 5;
        public static final int redLeftOutpostRight = 8;
        public static final int redRightOutpostMiddle = 2;
        public static final int redRightOutpostLeft = 11;
        public static final int redRearOutpostLeft = 3;
        public static final int redRearOutpostRight = 4;
        public static final int redHumanPlayerLeft = 13;
        public static final int redHumanPlayerRight = 14;
        public static final int redClimbLeft = 15;
        public static final int redClimbRight = 16;
    }
}