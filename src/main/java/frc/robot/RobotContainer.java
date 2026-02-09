package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ClimbAlign;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    
//Creates an XBOX Controller and Joystick object for our driver and operator
private final XboxController controller = new XboxController(0);
private final Joystick operator = new Joystick(1);

//These three variables represent the id's of the axis' on the Xbox Controller's sticks.
private final int translationAxis = XboxController.Axis.kLeftY.value;
private final int strafeAxis = XboxController.Axis.kLeftX.value;
private final int rotationAxis = XboxController.Axis.kRightX.value;

//----------------DRIVER CONTROLS and WHAT THEY DO---------------------------------------------------
//setHeading - sets the "front" of the robot to the direction it is facing. 
//controlToggle - when held, switches the robot into robotcentric. When facing a specific apriltag, 
//also switches the robot into aimAssist mode as well.
//brakeToggle !!!NOT IMPLEMENTED YET!!! - when held, slows the robot's drive down for precision inputs
//alignToClimb !!!PROOF OF CONCEPT!!! - when held, moves the robot to a position on the field based on april tag location
//zeroGyro !!!PLEASE REMOVE!!!
private final JoystickButton setHeading = new JoystickButton(controller, XboxController.Button.kY.value);
private final JoystickButton controlToggle = new JoystickButton(controller, XboxController.Button.kX.value);
private final JoystickButton brakeToggle = new JoystickButton(controller, XboxController.Button.kB.value);
private final JoystickButton alignToClimb = new JoystickButton(controller, XboxController.Button.kStart.value);
private final Trigger zeroGyro = new POVButton(controller, 180); // Marked for removal


//----------------OPERATOR CONTROLS and WHAT THEY DO-------------------------------------------------
//fireShooters !!!SUBSYSTEM DOESN'T EXIST ON CURRENT BASE!!! - fires the shooters simeltaneously
private final JoystickButton fireShooters = new JoystickButton(operator, 1);


//Creates a selector so that we can choose from a supply of autons built in PathPlanner
private final SendableChooser<Command> autoChooser;

/*Subsystems */
//s_swerve - Our Swerve Drive based off Neo Motors
//s_optics - Our camera system powered by PhotonVision on an OrangePI
private final Swerve s_swerve = new Swerve();
private final Vision s_optics = new Vision("MainCam");

public RobotContainer() {

    //Sets our s_swerve to always be following the TeleopSwerve command when not being used by any other systems
    s_swerve.setDefaultCommand(
        new TeleopSwerve(
            s_swerve,
            controller, 
            () -> controller.getRawAxis(translationAxis), 
            () -> controller.getRawAxis(strafeAxis), 
            () -> controller.getRawAxis(rotationAxis), 
            () -> false)
    );


    //-----------------Autonomous Command Setup---------------------------------------------
    //Creates commands to be used in PathPlanner for AUTONOMOUS ONLY
    //zeroGyro !!!CAN BE REMOVED!!!
    //setHeading - sets the heading of the robot
    //alignToClimb !!!PROOF OF CONCEPT!!! - moves the robot to a position on the field based on april tags
    NamedCommands.registerCommand("zeroGyro", new InstantCommand(() -> s_swerve.zeroGyro()));
    NamedCommands.registerCommand("setHeading", Commands.runOnce(() -> {
        double angle = s_swerve.isBlueAlliance() ? Math.PI : 0;
        s_swerve.adjustGyro(angle);
        Pose2d current_pose = s_swerve.getPose();
        Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
        s_swerve.resetOdometry(pose);
    }, s_swerve));
    NamedCommands.registerCommand("alignToClimb", new ClimbAlign(s_swerve, s_optics, 0, 67));


    //Builds an auton using a name. This name should match an auton in PathPlanner
    //Places the auton chooser in smart dashboard for easy access
    autoChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //Configures Button Bindings for TELEOP ONLY
    configureButtonBindings();
}

//Used by robot.java to get the autonomous path selected
public Command getAutonomousCommand(){
    return autoChooser.getSelected();
}

//Configures Button Bindings for TELEOP ONLY
private void configureButtonBindings() {
    //Can be removed
    zeroGyro.onTrue(new InstantCommand(() -> s_swerve.zeroHeading(), s_swerve));

    //Sets the heading
    setHeading.onTrue(Commands.runOnce(() -> {
        double angle = s_swerve.isBlueAlliance() ? Math.PI : 0;
        s_swerve.adjustGyro(angle);
        Pose2d current_pose = s_swerve.getPose();
        Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
        s_swerve.resetOdometry(pose);
    }, s_swerve));
     
    //Switches to RobotCentric/AimAssist
    controlToggle.whileTrue(new TeleopSwerve(
        s_swerve,
        controller, 
        () -> controller.getRawAxis(translationAxis),
        () -> controller.getRawAxis(strafeAxis),
        () -> s_optics.getYawVal()/40,
        () -> true)
    );

    //Aligns to point on the field using parameters (swerve, vision, x inches from tag, y inches from tag)
    alignToClimb.whileTrue(new ClimbAlign(s_swerve, s_optics, -10, 67));
    
}
}
