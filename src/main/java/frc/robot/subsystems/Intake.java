package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    // Creates a sparkmax object for the intake
    private SparkMax intake;


    // Percent output of the intake
    private double output = 0.80;

    // Creates a blank config object
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();


    public Intake() {
        // Creates the intake based on the arguments passed
        intake = new SparkMax(6, MotorType.kBrushless);


        // methods to configure the same sparkmaxes
        configIntake();

    }

    private void configIntake() {
        intakeConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
                .outputRange(-1, 1);
        intakeConfig.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(Constants.sparkMaxCurrentLimit)
                .inverted(false);
        intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    

    // Stops the intake
    public void stopIntake() {
        intake.stopMotor();
    }

    // Runs intake forward
    public void runIntakeFwd() {
        intake.set(output);
    }

    // Runs intake backward (In the event of a blockage)
    public void runIntakeBwd() {
        intake.set(-output);
    }

    // Runs intake according to a custom output double (From -1 to 1).
    public void runIntakeCustom(double customOutput) {
        intake.set(customOutput);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Current Consumption", intake.getOutputCurrent());
    }
}
