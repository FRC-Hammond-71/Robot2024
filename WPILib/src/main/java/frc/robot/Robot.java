package frc.robot;

import frc.robot.commands.FaceAtCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Movement.DriveSubsystem;
import frc.robot.subsystems.Movement.SimulatedDriveSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Movement.ActualDriveSubsystem;

public class Robot extends TimedRobot {
	// ----------
	// Subsystems
	// ----------
	private DriveSubsystem m_drive;
	private ArmSubsystem m_arm;

	private NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("Limelight");

	private static final String kDefaultAuto = "Cone DE Auto";
	private static final String kCustomAuto = "Cube DE Auto";
	private final SendableChooser<String> m_chooser = new SendableChooser<String>();

	@Override
	public void simulationInit() {
		System.out.println("Robot is running in simulation!");
	}

	@Override
	public void robotInit() {

		this.m_drive = RobotBase.isReal() ? new ActualDriveSubsystem() : new SimulatedDriveSubsystem();
		this.m_arm = new ArmSubsystem();

		CameraServer.startAutomaticCapture();

		Shuffleboard.getTab("Movement").add(m_drive);
		Shuffleboard.getTab("Automation").add(m_drive.FollowPathByName("Test Path"));
		// Shuffleboard.getTab("General").add(CommandScheduler.getInstance());

		// Limelight initiation Code
		LLTable.getEntry("camMode").setNumber(0);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		if (m_drive.DriverController.getPOV() == 180){
			CommandScheduler.getInstance().disable();
			m_drive.Stop();
		}
	}

	@Override
	public void teleopInit() 
	{
	}

	@Override
	public void teleopExit() 
	{
	}

	@Override
	public void disabledInit() 
	{
		
	}

	@Override
	public void disabledPeriodic() {
		this.m_drive.Stop();
	}
}
