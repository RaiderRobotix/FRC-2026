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
import frc.robot.commands.VisionShooter;
import frc.robot.subsystems.Gate;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    // Creates an XBOX Controller and Joystick object for our driver and operator
    private final XboxController controller = new XboxController(0);
    private final Joystick operator = new Joystick(1);

    // These three variables represent the id's of the axis' on the Xbox
    // Controller's sticks.
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    // ----------------DRIVER CONTROLS and WHAT THEY DO------------------------------------
    // setHeading - sets the "front" of the robot to the direction it is facing.
    // controlToggle - when held, switches the robot into robotcentric. When facing
    // a specific apriltag,
    // also switches the robot into aimAssist mode as well.
    // brakeToggle !!!NOT IMPLEMENTED YET!!! - when held, slows the robot's drive
    // down for precision inputs
    // alignToClimb !!!PROOF OF CONCEPT!!! - when held, moves the robot to a
    // position on the field based on april tag location
    // zeroGyro !!!PLEASE REMOVE!!!
    private final JoystickButton setHeading = new JoystickButton(controller, XboxController.Button.kY.value);
    private final JoystickButton controlToggle = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    private final JoystickButton brakeToggle = new JoystickButton(controller, XboxController.Button.kB.value);
    private final JoystickButton alignToClimb = new JoystickButton(controller, XboxController.Button.kStart.value);
    private final Trigger zeroGyro = new POVButton(controller, 180); // Marked for removal

    // ----------------OPERATOR CONTROLS and WHAT THEY DO-----------------------
    // fireShooters - fires the shooters simeltaneously
    // runIntake - runs the Intake subsystem forwards
    
    //private final JoystickButton fireShooters = new JoystickButton(operator, 1);
    private final JoystickButton changeSpeed = new JoystickButton(operator, 2);
    private final JoystickButton runIntake = new JoystickButton(operator, 8);
    private final JoystickButton runIntakeBwd = new JoystickButton(operator, 7);
    
    private final JoystickButton hopperOut = new JoystickButton(operator, 3);
    private final JoystickButton hopperIn = new JoystickButton(operator, 5);
    private final JoystickButton spin = new JoystickButton(operator, 12);
    private final JoystickButton gateUp = new JoystickButton(operator, 6);
    private final JoystickButton gateDown = new JoystickButton(operator, 4);
    private final JoystickButton customShot = new JoystickButton(operator, 1);
    // Creates a selector so that we can choose from a supply of autons built in
    // PathPlanner
    private final SendableChooser<Command> autoChooser;

    /* Subsystems */
    // s_swerve - Our Swerve Drive based off Neo Motors
    // s_optics - Our camera system powered by PhotonVision on an OrangePI
    // s_shooter - Our Shooters powered by 2 Neo Motors 
    // s_intake - Our Intake powered by a Neo Motor

    private final Swerve s_swerve = new Swerve();
    private final Vision s_optics = new Vision("Cam1");
    private final Shooter s_shooter = new Shooter();
    private final Intake s_intake = new Intake();
    private final Hopper s_hopper = new Hopper();
    private final Spindexer s_spindexer = new Spindexer();
    private final Gate s_gate = new Gate();

    public RobotContainer() {

        // Sets our s_swerve to always be following the TeleopSwerve command when not
        // being used by any other systems
        s_swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_swerve,
                        controller,
                        () -> -controller.getRawAxis(translationAxis),
                        () -> -controller.getRawAxis(strafeAxis),
                        () -> -controller.getRawAxis(rotationAxis),
                        () -> false));
        
        // -----------------Autonomous Command
        // Setup---------------------------------------------
        // Creates commands to be used in PathPlanner for AUTONOMOUS ONLY
        // zeroGyro !!!CAN BE REMOVED!!!
        // setHeading - sets the heading of the robot
        // alignToClimb !!!PROOF OF CONCEPT!!! - moves the robot to a position on the
        // field based on april tags
        NamedCommands.registerCommand("startIntake", new InstantCommand(() -> s_intake.runIntakeFwd()));
        NamedCommands.registerCommand("stopIntake", new InstantCommand(() -> s_intake.stopIntake()));
        NamedCommands.registerCommand("hopperOut", new InstantCommand(() -> s_hopper.runHopperOut()));
        NamedCommands.registerCommand("hopperIn", new InstantCommand(() -> s_hopper.runHopperIn()));
        NamedCommands.registerCommand("hopperStop", new InstantCommand(() -> s_hopper.stopHopper()));
        NamedCommands.registerCommand("startShooters", new InstantCommand(() -> s_shooter.runShootersFwd()));
        NamedCommands.registerCommand("stopShooters", new InstantCommand(() -> s_shooter.stopShooters()));
        NamedCommands.registerCommand("runAgitator", new InstantCommand(() -> s_spindexer.runSpindexer()));
        NamedCommands.registerCommand("stopAgitator", new InstantCommand(() -> s_spindexer.stopSpindexer()));
        NamedCommands.registerCommand("zeroGyro", new InstantCommand(() -> s_swerve.zeroGyro()));
        NamedCommands.registerCommand("gateUp", new InstantCommand(() -> s_gate.rungateFwd()));
        NamedCommands.registerCommand("gateDown", new InstantCommand(() -> s_gate.rungateBwd()));   
        NamedCommands.registerCommand("setHeading", Commands.runOnce(() -> {
            double angle = s_swerve.isBlueAlliance() ? Math.PI : 0;
            s_swerve.adjustGyro(angle);
            Pose2d current_pose = s_swerve.getPose();
            Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
            s_swerve.resetOdometry(pose);
        }, s_swerve));
        /*NamedCommands.registerCommand("visionTest", new TeleopSwerve(s_swerve,
                controller,
                () -> controller.getRawAxis(translationAxis),
                () -> controller.getRawAxis(strafeAxis),
                () -> s_optics.getYawVal() / 40,
                () -> true));
        */
        //NamedCommands.registerCommand("alignToClimb", new ClimbAlign(s_swerve, s_optics, 0, 67));
        /*NamedCommands.registerCommand("rotateToTag", Commands.runOnce(() -> {
            double angle = s_swerve.isBlueAlliance() ? Math.PI : 0;
            s_swerve.adjustGyro(angle);
            Pose2d current_pose = s_swerve.getPose();
            Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
            s_swerve.resetOdometry(pose);
        }, s_swerve));
        */
        // Builds an auton using a name. This name should match an auton in PathPlanner
        // Places the auton chooser in smart dashboard for easy access
        autoChooser = AutoBuilder.buildAutoChooser("rotateTest");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configures Button Bindings for TELEOP ONLY
        configureButtonBindings();
    }

    // Used by robot.java to get the autonomous path selected
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // Configures Button Bindings for TELEOP ONLY
    private void configureButtonBindings() {
        // Can be removed
        zeroGyro.onTrue(new InstantCommand(() -> s_swerve.zeroHeading(), s_swerve));

        brakeToggle.whileTrue(new TeleopSwerve(
        s_swerve,
        controller,
        () -> -(controller.getRawAxis(translationAxis)/2),
        () -> -(controller.getRawAxis(strafeAxis)/2),
        () -> -(controller.getRawAxis(rotationAxis)/2),
        () -> false
        ));
        // Sets the heading
        setHeading.onTrue(Commands.runOnce(() -> {
            double angle = 0;
            s_swerve.adjustGyro(angle);
            Pose2d current_pose = s_swerve.getPose();
            Pose2d pose = new Pose2d(current_pose.getX(), current_pose.getY(), new Rotation2d(angle));
            s_swerve.resetOdometry(pose);
        }, s_swerve));

        
        
        changeSpeed.whileTrue(new StartEndCommand(() -> s_shooter.runShootersBwd(), () -> s_shooter.stopShooters()));

        runIntake.whileTrue(new StartEndCommand(() -> s_intake.runIntakeFwd(), () -> s_intake.stopIntake()));
        runIntakeBwd.whileTrue(new StartEndCommand(() -> s_intake.runIntakeBwd(), () -> s_intake.stopIntake()));
        
        hopperIn.whileTrue(new StartEndCommand(() -> s_hopper.runHopperIn(), () -> s_hopper.stopHopper()));
        hopperOut.whileTrue(new StartEndCommand(() -> s_hopper.runHopperOut(), () -> s_hopper.stopHopper()));
        
        spin.whileTrue(new RunCommand(() -> s_spindexer.runSpindexer(), s_spindexer));
        spin.whileFalse(new RunCommand(() -> s_spindexer.stopSpindexer(), s_spindexer));
        
        gateUp.onTrue(new InstantCommand(() -> s_gate.rungateFwd()));
        gateDown.onTrue(new InstantCommand(() -> s_gate.rungateBwd()));
        // Switches to RobotCentric/AimAssist
        
        controlToggle.whileTrue(new TeleopSwerve(
                s_swerve,
                controller,
                () -> -controller.getRawAxis(translationAxis),
                () -> -controller.getRawAxis(strafeAxis),
                () -> -s_optics.getYawVal() / 40,
                () -> true));
        
        // Aligns to point on the field using parameters (swerve, vision, x inches from
        // tag, y inches from tag)
        //alignToClimb.whileTrue(new ClimbAlign(s_swerve, s_optics, -10, 67));
        

        
        
        customShot.whileTrue(new VisionShooter(s_optics, s_swerve, s_shooter));
        customShot.whileFalse(new InstantCommand(() -> s_shooter.stopShooters()));
    }
}
