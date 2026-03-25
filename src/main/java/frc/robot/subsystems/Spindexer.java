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
import java.util.Timer;

public class Spindexer extends SubsystemBase {
    // Creates a sparkmax object for the intake
    private SparkMax spinner;

    public int timer;

    // Percent output of the intake
    private double output = 0.45;

    // Creates a blank config object
    private SparkMaxConfig spinnerConfig = new SparkMaxConfig();


    public Spindexer() {
        // Creates the intake based on the arguments passed
        spinner = new SparkMax(9, MotorType.kBrushless);


        // methods to configure the same sparkmaxes
        configSpindexer();

    }

    private void configSpindexer() {
        spinnerConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
                .outputRange(-1, 1);
        spinnerConfig.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(Constants.sparkMaxCurrentLimit)
                .inverted(false);
        spinner.configure(spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    // Runs spindexer
    public void runSpindexer() {
        spinner.set(output);
    }


    public void stopSpindexer() {
        spinner.stopMotor();
    }

    @Override
    public void periodic(){
        timer++;
        if(timer == 50){
            
            output*=-1;
            timer = 0;
            
        }

        SmartDashboard.putNumber("Timer Value", timer);
        
    }
}
