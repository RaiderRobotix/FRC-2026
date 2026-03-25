package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Gate extends SubsystemBase {
    // Creates a sparkmax object for the gate
    private SparkMax gate;


    // Percent output of the gate
    private double output = 0.60;

    // Creates a blank config object
    private SparkMaxConfig gateConfig = new SparkMaxConfig();

    private RelativeEncoder gateEncoder;

    private SparkClosedLoopController gateController;

    private boolean isUp = false;


    public Gate() {
        // Creates the gate based on the arguments passed
        gate = new SparkMax(8, MotorType.kBrushless);


        // methods to configure the same sparkmaxes
        configGate();

        gateController = gate.getClosedLoopController();

        gateEncoder = gate.getEncoder();
    }

    private void configGate() {
        gateConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
                .outputRange(-1, 1);
        gateConfig.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(Constants.sparkMaxCurrentLimit)
                .inverted(false);
        gate.configure(gateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    

    // Stops the gate
    public void stopgate() {
        gate.stopMotor();
    }

    public void gateUp(){
        gateController.setReference(0, ControlType.kPosition);
    }

    // Runs gate open
    public void rungateFwd() {
        gate.set(output);
        isUp = true;
    }

    // Runs gate closed
    public void rungateBwd() {
        gate.set(-output);
        isUp = false;
    }

    

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Gate Rotation", gateEncoder.getPosition());
        SmartDashboard.putBoolean("Gate is UP", isUp);
    }

    
}
