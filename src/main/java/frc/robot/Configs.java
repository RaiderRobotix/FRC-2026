package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MotorConstants;

public class Configs {

        // Configuration class for MAX Swerve Modules
        public static final class MAXSwerveModule {
                // Driving Motor Configuration
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                // Turning Motor Configuration
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
                // CANCoder Configuration
                public static final CANcoderConfiguration swerveCANcoderconfig = new CANcoderConfiguration();

                // Static initializer block to set up configurations
                static {
                        double drivingFactor = MotorConstants.kWheelDiameterMeters * Math.PI
                                        / MotorConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double drivingVelocityFeedForward = 1 / MotorConstants.kDriveWheelFreeSpeedRps;

                        // Configure Driving Motor
                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(40);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor)
                                        .velocityConversionFactor(drivingFactor / 60.0);
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.04, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        // Configure Turning Motor
                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true);
                        turningConfig.encoder
                                        .positionConversionFactor((2 * Math.PI) / Constants.Swerve.angleGearRatio)
                                        .velocityConversionFactor((2 * Math.PI) / Constants.Swerve.angleGearRatio / 60);
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }

        }
}