package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
    private Vision vision;
    private Swerve drivetrain;
    // Motors and drive system
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
    private final XboxController m_controller = new XboxController(0);
    private final Timer m_timer = new Timer();
    private Command m_autonomousCommand;
    private RobotContainer m_RobotContainer;

    @Override
    public void robotInit() {
        drivetrain = new Swerve();
        vision = new Vision(drivetrain::addVisionMeasurement);

        SendableRegistry.addChild(m_robotDrive, m_leftDrive);
        SendableRegistry.addChild(m_robotDrive, m_rightDrive);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightDrive.setInverted(true);

        m_RobotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        vision.periodic();

        SmartDashboard.putNumber("Robot X Position", drivetrain.poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Robot Y Position", drivetrain.poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Robot Rotation",
                drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    /** This function is run once each time the robot enters autonomous mode. */
    @Override
    public void autonomousInit() {
        m_timer.restart();
        m_autonomousCommand = m_RobotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // // Drive for 2 seconds
        // if (m_timer.get() < 2.0) {
        // // Drive forwards half speed, make sure to turn input squaring off
        // m_robotDrive.arcadeDrive(0.5, 0.0, false);
        // } else {
        // m_robotDrive.stopMotor(); // stop robot
        // }
    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during teleoperated mode. */
    @Override
    public void teleopPeriodic() {
        m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
    }

}
