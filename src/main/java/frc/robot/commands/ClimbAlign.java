package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbAlign extends Command{
    private Swerve s_swerve;
    private Vision s_optics;
    private double xFromTag;
    private double yFromTag;

    //Method takes in your swerve object, vision object, and your xFromTag(Left is positive Right is negative) and yFromTag
    public ClimbAlign(Swerve s_swerve, Vision s_optics, double xFromTag, double yFromTag){
        this.s_swerve = s_swerve;
        this.s_optics = s_optics;
        this.xFromTag = xFromTag;
        this.yFromTag = yFromTag;
    }

    @Override
    public void execute(){
        double y_translation = (s_optics.getYawVal()-(Math.atan(xFromTag/yFromTag)*180/Math.PI))/40;
        double x_translation = (yFromTag-s_optics.getDistanceFromPitchVal())/100;
        s_swerve.drive(
        new Translation2d(x_translation, y_translation).times(Constants.kMaxSpeedMetersPerSecond), 
        (s_swerve.getHeading().getDegrees()/90) * Constants.kMaxAngularSpeed, 
        true, 
        true
        );
        
    }
}
