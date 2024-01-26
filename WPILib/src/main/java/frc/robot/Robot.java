package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControlSubsystem;
import frc.robot.subsystems.Movement.DriveSubsystem;
import frc.robot.subsystems.Movement.SimulatedDriveSubsystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Movement.ActualDriveSubsystem;

public class Robot extends TimedRobot {
	// ----------
	// Subsystems
	// ----------
	private DriveSubsystem m_drive;
	private ArmSubsystem m_arm;
	private ControlSubsystem m_control;

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
		this.m_control = new ControlSubsystem(m_drive, m_arm);

		CameraServer.startAutomaticCapture();

		// Limelight initiation Code
		LLTable.getEntry("camMode").setNumber(0);

		CommandScheduler.getInstance().unregisterSubsystem(m_control);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().registerSubsystem(this.m_control);
	}

	// @Override
	// public void teleopPeriodic()
	// {

	// }

	@Override
	public void teleopExit() {
		CommandScheduler.getInstance().unregisterSubsystem(this.m_control);
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
