package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private final XboxController controller = new XboxController(0);
    private final Joystick operator = new Joystick(1);

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(controller, XboxController.Button.kY.value);

    private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    private final Swerve s_swerve = new Swerve();

    public RobotContainer() {

        /*
         * s_swerve.setDefaultCommand(new RunCommand(
         * () -> s_swerve.drive(
         * -MathUtil.applyDeadband(controller.getLeftX(), Constants.stickDeadband),
         * -MathUtil.applyDeadband(controller.getLeftY(), Constants.stickDeadband),
         * -MathUtil.applyDeadband(controller.getRightX(), Constants.stickDeadband),
         * true),
         * s_swerve));
         * 
         */
        configureDriverControls();

        s_swerve.setDefaultCommand(new RunCommand(
                () -> s_swerve.drive(
                        controller.getRawAxis(translationAxis),
                        controller.getRawAxis(strafeAxis),
                        controller.getRawAxis(rotationAxis),
                        true),
                s_swerve));

        NamedCommands.registerCommand("zeroGyro", new InstantCommand(() -> s_swerve.zeroGyro()));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureButtonBindings();

        /* Subsystems */

        // Configure the button bindings ( what a useful comment)

        // configureOperatorControls();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureDriverControls() {

        /* Drive Controls */
        // this code sets up driving in teleop.

        double translationAxis = XboxController.Axis.kLeftY.value;
        double strafeAxis = XboxController.Axis.kLeftX.value;
        double rotationalAxis = XboxController.Axis.kRightX.value;

        JoystickButton zeroGyro = new JoystickButton(controller, XboxController.Button.kY.value);

    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_swerve.zeroHeading()));
    }
}