package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{
    private TalonFX hopperTalon;

    private double outputIn = 0.80;
    private double outputOut = 0.95;
    private double min = 1.5; // Furthest in the hopper is permitted to go
    private double max = 162.9; // Furthest out the hopper is permitted to go.

    private boolean disableUp;
    private boolean disableDown;
    
    public Hopper(){
        hopperTalon = new TalonFX(7);
        hopperTalon.setNeutralMode(NeutralModeValue.Brake);
        min = hopperTalon.getPosition().getValueAsDouble();
        max = max+min;
    }

    public void runHopperIn(){
         
        if(hopperTalon.getPosition().getValueAsDouble() < min){
            stopHopper();
        }else{
            
            hopperTalon.set(-outputIn);
        }
        
        hopperTalon.set(-outputIn);
    }

    public void runHopperOut(){
         
        if(hopperTalon.getPosition().getValueAsDouble() > max){
            stopHopper();
        }else{
            hopperTalon.set(outputOut);
            
        }
        
        hopperTalon.set(outputOut);
    }

    public void stopHopper(){
        hopperTalon.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hopper Offset", hopperTalon.getPosition().getValueAsDouble());
        
        if(hopperTalon.getPosition().getValueAsDouble() < min || hopperTalon.getPosition().getValueAsDouble() > max){
            stopHopper();
        }
            
    }

}
