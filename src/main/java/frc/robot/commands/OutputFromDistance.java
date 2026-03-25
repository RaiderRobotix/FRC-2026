package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class OutputFromDistance extends Command {
    private Swerve s_swerve;
    private Vision s_optics;
    private double output;

    public OutputFromDistance(Swerve s_swerve, Vision s_optics) {
        this.s_swerve = s_swerve;
        this.s_optics = s_optics;
    }
    /*
    @Override
    public void execute() {
        double dist = s_optics.getDistanceFromPitchVal();
        double a = Constants.gravity * Math.pow(dist + 47, 2);
        double b = Math.sin(2 * 67.2) * (dist + 47) - 2 * Math.pow(Math.cos(67.2), 2) * (72 - 36);
        double inchesPerSecond = Math.sqrt(a / b);
    }
        */
}
