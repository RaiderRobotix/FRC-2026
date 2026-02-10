// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;

// public class Shooter extends SubsystemBase{
//     //Creates two sparkmax objects for the left and right shooter
//     private SparkMax leftShooter;
//     // private SparkMax rightShooter;

//     //Percent output of the shooters
//     private double output = 0.1;

//     //Creates two blank config objects
//     private SparkMaxConfig leftConfig = new SparkMaxConfig();
//     // private SparkMaxConfig rightConfig = new SparkMaxConfig();

//     public Shooter(){
//         //Creates the shooters based on the arguments passed
//         leftShooter = new SparkMax(8, MotorType.kBrushless);
//         // rightShooter = new SparkMax(9, MotorType.kBrushless);

//         //methods to configure the same sparkmaxes
//         configLeftShooter();
//         // configRightShooter();
//     }
//     private void configLeftShooter(){
//         leftConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
//         .outputRange(-1, 1);
//         leftConfig.idleMode(IdleMode.kCoast)
//         .smartCurrentLimit(10)
//         .inverted(false);
//         leftShooter.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }
//     // private void configRightShooter(){
//     //     rightConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
//     //     .outputRange(-1, 1);
//     //     rightConfig.idleMode(IdleMode.kCoast)
//     //     .smartCurrentLimit(10)
//     //     .inverted(false);
//     //     rightShooter.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     // }

//     //Stops the shooters
//     public void stopShooters(){
//         leftShooter.stopMotor();
//         // rightShooter.stopMotor();
//     }

//     //Runs shooters forward
//     public void runShootersFwd(){
//         leftShooter.set(output);
//         // rightShooter.set(output);
//     }

//     //Runs shooters backwards (In the event of a blockage)
//     public void runShootersBwd(){
//         leftShooter.set(-output);
//         // rightShooter.set(-output);
//     }
// }

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.trajectoryConstants;

public class Flywheel extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    private final LinearFilter rpmFilter = LinearFilter.movingAverage(5);

    private double targetRPM = 0.0;

    public Flywheel(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        motor.clearFaults();

        encoder = motor.getEncoder();

        pid = new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
        pid.setTolerance(FlywheelConstants.kToleranceRPM);

        // Initialize feedforward (kV here is in V per RPM since we use RPM everywhere)
        feedforward = new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
    }

    @Override
    public void periodic() {
        double measuredRPM = getFilteredRPM();

        double ffVolts = feedforward.calculate(targetRPM);
        double pidVolts = pid.calculate(measuredRPM, targetRPM);

        // Combine feedforward and PID output into a single voltage command.
        // Clamp to the controller supply range to avoid rapid saturation and
        // possible sign-flipping from hitting limits.
        double volts = MathUtil.clamp(ffVolts + pidVolts, -12.0, 12.0);
        motor.setVoltage(volts);
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        pid.reset();
    }

    public void stop() {
        targetRPM = 0.0;
        motor.stopMotor();
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public double getFilteredRPM() {
        return rpmFilter.calculate(getRPM());
    }

    public boolean atSpeed() {
        return pid.atSetpoint();
    }

    public static double distanceToV0(double distance, double thetaDegrees) {
        double thetaRadians = Math.toRadians(thetaDegrees);
        double relativeHeight = trajectoryConstants.kTargetHeight - trajectoryConstants.kShooterHeight;
        DriverStation.reportWarning("Relative height: " + relativeHeight, false);
        double v0 = Math.sqrt(

                (trajectoryConstants.kGravity
                    *
                Math.pow(distance + trajectoryConstants.kTargetWidth, 2))
                /
                (Math.sin(2 * thetaRadians)
                    * (distance + trajectoryConstants.kTargetWidth)
                        - (2
                            * (Math.pow(Math.cos(thetaRadians), 2))
                            * (relativeHeight - trajectoryConstants.kTolerance)))

        );
        return v0;
    }

    public static double v0ToRPM(double v0) {
        // Convert linear velocity (in/s) to angular velocity (RPM)
        // Assuming a wheel radius of 3 inches (example), the circumference is 2 * pi *
        // r
        double wheelRadiusInches = 3.0;
        double wheelCircumferenceInches = 2 * Math.PI * wheelRadiusInches;
        double rpm = (v0 / wheelCircumferenceInches) * 60.0; // Convert from in/s to RPM
        return rpm;
    }
}
