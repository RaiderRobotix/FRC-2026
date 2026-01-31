package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.ModuleConstants;

public class MAXSwerveModule {
    // Motors
    private SparkMax m_drivingSpark;
    private SparkMax m_turningSpark;

    // Motor Encoders
    private RelativeEncoder m_drivingEncoder;
    private RelativeEncoder m_turningEncoder;

    // Configuration object containing motor IDs, offsets, etc.
    private final ModuleConstants moduleConstants;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private CANcoder angleEncoder;
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double rotval = 0;

    public MAXSwerveModule(ModuleConstants moduleConstants) {
        // Initialize motors and encoders using constants
        this.moduleConstants = moduleConstants;
        m_drivingSpark = new SparkMax(moduleConstants.drivermotorID0, MotorType.kBrushless);
        m_turningSpark = new SparkMax(moduleConstants.anglemotorID0, MotorType.kBrushless);

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        angleEncoder = new CANcoder(moduleConstants.camcoderID);
        configEncoders();

        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        m_chassisAngularOffset = this.moduleConstants.offset0;

        resetToAbsolute();

        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    }

    /**
     * Constants about this module like motor IDs and offsets.
     */
    public ModuleConstants getModuleConstants() {
        return moduleConstants;
    }
    private void configEncoders() {
        angleEncoder.getConfigurator().apply(Configs.MAXSwerveModule.swerveCANcoderconfig);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_drivingEncoder.setPosition(0);

        m_turningEncoder = m_turningSpark.getEncoder();

    }
    /**
     * Sets the turning encoder to the absolute position of the CANCoder minus the
     * chassis angular offset.
     */
    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - m_chassisAngularOffset;
        m_turningEncoder.setPosition(absolutePosition);
    }
    /**
     * 
     * @return The current position of the swerve module as a SwerveModulePosition.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), getAngle());
    }
    /**
     * 
     * @return The current position of the turning encoder in degrees.
     */
    public double getEncoderPosition() {
        return m_turningEncoder.getPosition();
    }
    /**
     * 
     * @return The current state of the swerve module as a SwerveModuleState.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), getAngle());
    }
    /**
     * Sets the desired state of the swerve module.
     * 
     * @param desiredState The desired state to set.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        setAngle(correctedDesiredState);
        setSpeed(correctedDesiredState, true);
    }
    /**
     * 
     * @return The current angle of the swerve module as a Rotation2d.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(m_turningEncoder.getPosition());
    }
    /**
     * 
     * @return The absolute position of the CANCoder as a Rotation2d.
     */
    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }
    /**
     * Sets the angle of the swerve module.
     * 
     * @param desiredState The desired state to set the angle to.
     */
    private void setAngle(SwerveModuleState desiredState) {

        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.kMaxAngularSpeed * 0.01)) {
            m_turningSpark.stopMotor();
            return;
        }
        rotval = desiredState.angle.getRadians();

        m_turningClosedLoopController.setReference(rotval, ControlType.kPosition);
    }
    /**
     * Sets the speed of the swerve module.
     * 
     * @param desiredState The desired state to set the speed to.
     * @param isOpenLoop   Whether to use open-loop control or closed-loop control.
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.kMaxSpeedMetersPerSecond;
            m_drivingSpark.set(percentOutput);
            return;
        }
        m_drivingClosedLoopController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    }
    /**
     * 
     * @return The current rotational value setpoint of the swerve module in
     *         radians.
     */
    public double getRotVal() {
        return rotval;
    }
}
