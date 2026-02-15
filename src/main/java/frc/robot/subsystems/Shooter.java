package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    //Creates two sparkmax objects for the left and right shooter
    private SparkMax leftShooter;
    private SparkMax rightShooter;

    //Percent output of the shooters
    private double output = 0.1;

    //Creates two blank config objects
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    public Shooter(){
        //Creates the shooters based on the arguments passed
        leftShooter = new SparkMax(4, MotorType.kBrushless);
        rightShooter = new SparkMax(5, MotorType.kBrushless);

        //methods to configure the same sparkmaxes
        configLeftShooter();
        configRightShooter();
    }
    private void configLeftShooter(){
        leftConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
        .outputRange(-1, 1);
        leftConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10)
        .inverted(false);
        leftShooter.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    private void configRightShooter(){
        rightConfig.closedLoop.pidf(1.0, 0.0, 0.0, 0.0)
        .outputRange(-1, 1);
        rightConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10)
        .inverted(false);
        rightShooter.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //Stops the shooters
    public void stopShooters(){
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    //Runs shooters forward
    public void runShootersFwd(){
        leftShooter.set(output);
        rightShooter.set(output);
    }

    //Runs shooters backwards (In the event of a blockage)
    public void runShootersBwd(){
        leftShooter.set(-output);
        rightShooter.set(-output);
    }

    //Runs shooters according to a custom output double (From -1 to 1). Most likely used in junction with vision
    public void runShootersCustom(double customOutput){
        leftShooter.set(customOutput);
        rightShooter.set(customOutput);
    }
}
