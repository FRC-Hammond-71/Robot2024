// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// IF ERROR CODE 2 --> CHECK THE TINY SWITCH
package frc.robot;

import frc.robot.LimelightHelpers;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.AbsoluteEncoder;


import java.util.AbstractQueue;
import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;




public class Robot extends TimedRobot {

  private XboxController m_driverController = new XboxController(0);
  private XboxController m_operatorController = new XboxController(1);

  private DriveSubsystem m_drive;
  private ArmSubsystem m_arm;

  /*private AbsoluteEncoder m_armAbsoluteEncoder;*/

  private NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("Limelight"); ;

  AHRS ahrs;

  private final double kArmTick2Deg = 32.667;
  private final double kDriveTick2Feet = 1;

  private static final String kDefaultAuto = "Cone DE Auto";
  private static final String kCustomAuto = "Cube DE Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private double startTime;

  private Command InputBlockingCommand;


  public Robot()
  {
    super();
    this.m_drive = new DriveSubsystem();
    this.m_arm = new ArmSubsystem();
  }

  @Override
  public void robotInit() {
        
    // m_robotDrive.setDeadband(0.05);

    m_chooser.setDefaultOption("Cone DE Auto", kDefaultAuto);
    m_chooser.addOption("Cube DE Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,980);
    m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,-980);*/

    
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */

      ahrs = new AHRS(SPI.Port.kMXP); 
      CameraServer.startAutomaticCapture();

      //Limelight initiation Code
      LLTable.getEntry("camMode").setNumber(0);

      CommandScheduler.getInstance().registerSubsystem(m_drive);
      CommandScheduler.getInstance().registerSubsystem(m_arm);
  }

  @Override
  public void robotPeriodic() {
    /*SmartDashboard.putNumber("Arm Encoder Position", m_armAbsoluteEncoder.getPosition());*/
    // SmartDashboard.putNumber("Left Lead Motor Encoder", m_leftLeadEncoder.getPosition()*kDriveTick2Feet);
    // SmartDashboard.putNumber("Right Lead Motor Encoder", m_rightLeadEncoder.getPosition()*kDriveTick2Feet);
    // SmartDashboard.putNumber("Left Follow Motor Encoder", m_leftFollowEncoder.getPosition()*kDriveTick2Feet);
    // SmartDashboard.putNumber("Right Follow Motor Encoder", m_rightFollowEncoder.getPosition()*kDriveTick2Feet);
    // SmartDashboard.putNumber("Right Follow Motor Encoder", m_rightFollowEncoder.getPosition()*kDriveTick2Feet);
    
    /* Display 6-axis Processed Angle Data                                      */
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

    SmartDashboard.putString("Chassis Speeds", this.m_drive.GetChassisSpeeds().toString());
    SmartDashboard.putNumber("Arm Position", this.m_arm.GetArmRotation());

   
    //read values periodically
    NetworkTableEntry Tx = LLTable.getEntry("tx");
    NetworkTableEntry Ty = LLTable.getEntry("ty");
    NetworkTableEntry Ta = LLTable.getEntry("ta");
    NetworkTableEntry Tv = LLTable.getEntry("tv");

    //post to smart dashboard periodically
    SmartDashboard.putBoolean("Targets", LimelightHelpers.getTV("limelight"));
    SmartDashboard.putNumber("TargetX", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("TargetY", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("TargetArea", LimelightHelpers.getTA("limelight"));
    SmartDashboard.putNumber("ID", LimelightHelpers.getFiducialID("limelight"));

    SmartDashboard.putString("Input Blocking Command", this.InputBlockingCommand == null ? "None" : this.InputBlockingCommand.getName());
  
    if (this.InputBlockingCommand != null && this.InputBlockingCommand.isFinished())
    {
      this.InputBlockingCommand = null;
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    if (this.InputBlockingCommand == null)
    {
      // Run our teleop input handle.
      
      m_drive.Drive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX()*0.7);

      double armPower = -m_operatorController.getLeftY();
      m_arm.ArmMotor.set(armPower * 0.7);
      
      if (this.m_operatorController.getAButton() == true)
      {
        this.InputBlockingCommand = m_arm.MoveTo(103, 40).withTimeout(5);
        this.InputBlockingCommand.schedule();

        System.out.print("Moving up!");
      }
    }

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    double startTime = Timer.getFPGATimestamp();
  }

  // @Override
  // public void testPeriodic() {
  //   double time = Timer.getFPGATimestamp(); 
    
  //   if(time - startTime < 1) {
  //     m_intakeMotor.set(1);
  //   } else {
  //     m_armMotor.set(0);
  //   }
  // }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  


}

