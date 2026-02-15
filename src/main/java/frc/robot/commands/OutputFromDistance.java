package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class OutputFromDistance extends Command{
    private Swerve s_swerve;
    private Vision s_optics;
    private double output;
    
    public OutputFromDistance(Swerve s_swerve, Vision s_optics){
        this.s_swerve = s_swerve;
        this.s_optics = s_optics;
    }

    @Override
    public void execute(){
        double dist = s_optics.getDistanceFromPitchVal();
        double a = Constants.gravity*Math.pow(dist+47, 2);
        double b = Math.sin(2*67.2)*(dist+47) - 2*Math.pow(Math.cos(67.2), 2)*(72-36);
        double inchesPerSecond = Math.sqrt(a/b);
    }
}
