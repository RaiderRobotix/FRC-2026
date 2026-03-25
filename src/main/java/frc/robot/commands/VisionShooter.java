package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class VisionShooter extends Command{
    private Vision s_optics;
    private Swerve s_swerve;
    private Shooter s_shooter;
    private double distance;
    private double max_speed = 70;
    private double min_speed = 50;
    private double max_range = 3.03;
    private double min_range = 1.55;
    

    public VisionShooter(Vision s_optics, Swerve s_swerve, Shooter s_shooter){
        this.s_optics = s_optics;
        this.s_swerve = s_swerve;
        this.s_shooter = s_shooter;
        distance = s_optics.getDistance();
    }

    @Override
    public void execute(){
        if(distance <= max_range && distance >= min_range){
            double percent = (distance - min_range)/(max_range - min_range);
            double new_speed = percent*(max_speed - min_speed)+min_speed;
            s_shooter.runShootersCustom(new_speed*0.01);
            System.out.println(new_speed);
        }else{
            s_shooter.runShootersFwd();
        }
    }
}
