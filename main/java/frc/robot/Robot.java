// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// IF ERROR CODE 2 --> CHECK THE TINY SWITCH
package frc.robot;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;




public class Robot extends TimedRobot {
  private CANSparkMax m_leftLeadMotor;
  private CANSparkMax m_rightLeadMotor;
  private CANSparkMax m_leftFollowMotor;
  private CANSparkMax m_rightFollowMotor;

  private CANSparkMax m_armMotor;
  private CANSparkMax m_intakeMotor;

  private DifferentialDrive m_robotDrive;

  private XboxController m_driverController;
  private XboxController m_operatorController;

  private RelativeEncoder m_leftLeadEncoder;
  private RelativeEncoder m_rightLeadEncoder;
  private RelativeEncoder m_leftFollowEncoder;
  private RelativeEncoder m_rightFollowEncoder;
  private RelativeEncoder m_armEncoder;
  private RelativeEncoder m_intakeEncoder;

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


  @Override
  public void robotInit() {
   

    
    // motors and encoders
    m_leftLeadMotor = new CANSparkMax(5,MotorType.kBrushless);
    m_rightLeadMotor = new CANSparkMax(4,MotorType.kBrushless);
    m_leftFollowMotor = new CANSparkMax(3,MotorType.kBrushless);
    m_rightFollowMotor = new CANSparkMax(2,MotorType.kBrushless);

    m_armMotor = new CANSparkMax(6,MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(7, MotorType.kBrushless);

    m_leftLeadMotor.restoreFactoryDefaults();
    m_rightLeadMotor.restoreFactoryDefaults();
    m_leftFollowMotor.restoreFactoryDefaults();
    m_rightFollowMotor.restoreFactoryDefaults();
    m_armMotor.restoreFactoryDefaults();
    m_intakeMotor.restoreFactoryDefaults();

    m_rightLeadMotor.setInverted(true);

    m_leftFollowMotor.follow(m_leftLeadMotor);
    m_rightFollowMotor.follow(m_rightLeadMotor);

    m_leftLeadEncoder = m_leftLeadMotor.getEncoder();
    m_rightLeadEncoder = m_rightLeadMotor.getEncoder();
    m_leftFollowEncoder = m_leftFollowMotor.getEncoder();
    m_rightFollowEncoder = m_rightFollowMotor.getEncoder();

    m_armEncoder = m_armMotor.getEncoder();
    m_intakeEncoder = m_intakeMotor.getEncoder();
    /*m_armAbsoluteEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);*/
    

    m_robotDrive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);

    m_driverController = new XboxController(0);
    m_operatorController = new XboxController(1);

    m_robotDrive.setDeadband(0.05);

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
  
 


}


  @Override
  public void robotPeriodic() {
    /*SmartDashboard.putNumber("Arm Encoder Position", m_armAbsoluteEncoder.getPosition());*/
    SmartDashboard.putNumber("Left Lead Motor Encoder", m_leftLeadEncoder.getPosition()*kDriveTick2Feet);
    SmartDashboard.putNumber("Right Lead Motor Encoder", m_rightLeadEncoder.getPosition()*kDriveTick2Feet);
    SmartDashboard.putNumber("Left Follow Motor Encoder", m_leftFollowEncoder.getPosition()*kDriveTick2Feet);
    SmartDashboard.putNumber("Right Follow Motor Encoder", m_rightFollowEncoder.getPosition()*kDriveTick2Feet);
    SmartDashboard.putNumber("Right Follow Motor Encoder", m_rightFollowEncoder.getPosition()*kDriveTick2Feet);
   

    /* Display 6-axis Processed Angle Data                                      */
    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

   
  
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
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    m_leftLeadEncoder.setPosition(0);
    m_rightLeadEncoder.setPosition(0);
    m_rightFollowEncoder.setPosition(2000);
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    
  }

  @Override
  public void autonomousPeriodic() {
   double time = Timer.getFPGATimestamp();
   double leftPosition = m_leftLeadEncoder.getPosition() * kDriveTick2Feet;
   double rightPosition = m_rightLeadEncoder.getPosition() * kDriveTick2Feet;
   double thirdDistance = m_rightFollowEncoder.getPosition();
   double distance = (leftPosition + rightPosition) / 2;
   double imuRoll = ahrs.getRoll();
   int eventID = 0;
   SmartDashboard.putNumber("EventID",     eventID);
  //AUTO: score cone, grab block, balance
   switch (m_autoSelected) {
    case kDefaultAuto:
     if (time - startTime < 1) {
      m_armMotor.set(-0.5);
  
     } else if (time - startTime < 1.5) {
      m_armMotor.set(0);
      m_intakeMotor.set(-0.2);
  //score cone
     } else if (time - startTime < 3) {
      m_intakeMotor.set(0);
     // eventID += 1;
      m_armMotor.set(0.5);
      m_robotDrive.tankDrive(0.5, 0.5);
  
     } else if (time - startTime < 4) {
      m_armMotor.set(0);
      m_robotDrive.tankDrive(0.5, 0.5);
  //grab blocc
     } else if (time - startTime < 5.6) {
      m_armMotor.set(0.5);
      m_robotDrive.tankDrive(0.5, 0.5);

     } else if (time - startTime < 6.5) {
        m_robotDrive.tankDrive(0.5, 0.5);
        m_armMotor.set(0);
        m_intakeMotor.set(-1);
  
     } else if (time - startTime < 9) {
        m_robotDrive.tankDrive(-0.5, -0.5);
       // eventID += 1;
        m_armMotor.set(-0.4);
        m_intakeMotor.set(0);

      } else if (time - startTime < 9.5) {
        m_robotDrive.tankDrive(-0.5, -0.5);
       // eventID += 1;
        m_armMotor.set(0);
        //m_intakeMotor.set(0);


  //BALANCE ==================================================================
     } else if (imuRoll < -11) {
      m_robotDrive.tankDrive(-0.3, -0.3);
     // m_armMotor.set(0);
     } else if (imuRoll < 11 && imuRoll > 0) {
      m_robotDrive.tankDrive(0, 0);
  
     } else if (imuRoll < -11) {
      m_robotDrive.tankDrive(-0.3, -0.3);
  
     } else if (imuRoll > 0 && imuRoll < 11) {
      m_robotDrive.tankDrive(0, 0);
  
     } else if (imuRoll > 11) {
      m_robotDrive.tankDrive(0.3, 0.3);
   //FAILSAFE FOR ARM ========================================================
     } else if (time - startTime < 12)  {
         m_armMotor.set(0);
     }
      break;
    case kCustomAuto:
    default:
    if (time - startTime < 1) {
      m_armMotor.set(-0.7);
  
     } else if (time - startTime < 1.5) {
      m_armMotor.set(0);
      m_intakeMotor.set(1);
  
     } else if (time - startTime < 3) {
      m_intakeMotor.set(0);
      m_armMotor.set(0.7);
  
     } else if (time - startTime < 4) {
      m_armMotor.set(0);
  
     } else if (time - startTime < 8.5) {
      m_robotDrive.tankDrive(0.5, 0.5);
  
     } else if (time - startTime < 11) {
      m_robotDrive.tankDrive(-0.5, -0.5);
  
     } else if (imuRoll < -11) {
      m_robotDrive.tankDrive(-0.3, -0.3);
  
     } else if (imuRoll < 11 && imuRoll > 0) {
      m_robotDrive.tankDrive (0, 0);
  
     } else if (imuRoll < -11) {
      m_robotDrive.tankDrive (-0.3, -.3);
  
     } else if (imuRoll > 0 && imuRoll < 11) {
      m_robotDrive.tankDrive (0, 0);
  
     } else if (imuRoll > 11) {
      m_robotDrive.tankDrive (0.3, 0.3);
     }
      break;
  // FIRST AUTO CODE ====================================================
  /*  if (distance < 37) {
    m_robotDrive.tankDrive(0.4, 0.4);
   } else if (imuRoll > 11) {
    m_robotDrive.tankDrive(0.3, 0.3);
   } else if (imuRoll < 11 && imuRoll > 0) {
    m_robotDrive.tankDrive (0, 0);
   } else if (imuRoll < 0) {
    m_robotDrive.tankDrive (-0.25, -0.25);
   } else if (imuRoll > 0) {
    m_robotDrive.tankDrive (0, 0);
   }
   +/
  
   // SECOND AUTO CODE ==================================================
   /* if (time - startTime < 1) {
    m_armMotor.set(-0.5);

   } else if (time - startTime < 1.5) {
    m_armMotor.set(0);
    m_intakeMotor.set(1);

   } else if (time - startTime < 3) {
    m_intakeMotor.set(0);
    m_armMotor.set(0.5);

   } else if (time - startTime < 4) {
    m_armMotor.set(0);

   } else if (time - startTime < 6.1) {
    m_leftLeadMotor.set(0.3);
    m_rightLeadMotor.set(0.3);
  
   } else if (time - startTime < 6.2) {
    m_leftLeadMotor.set(-0.5);
    m_rightLeadMotor.set(-0.5);
    /*m_leftFollowMotor.set(0.5);
    m_rightFollowMotor.set(0.5);*/

    // THIRD AUTO CODE =======================================================
   /*if (time - startTime < 1) {
    m_armMotor.set(-0.5);

   } else if (time - startTime < 1.5) {
    m_armMotor.set(0);
    m_intakeMotor.set(1);

   } else if (time - startTime < 3) {
    m_intakeMotor.set(0);
    m_armMotor.set(0.5);

   } else if (time - startTime < 4) {
    m_armMotor.set(0);

/*time - startTime < 6.2

   } else if (IMU_Roll > -12.5) {
    m_leftLeadMotor.set(0.3);
    m_rightLeadMotor.set(0.3);
  
 /*time - startTime < 6.4
   } else if () {
    m_leftLeadMotor.set(-0.15);
    m_rightLeadMotor.set(0.15); 
    
   } else if (time - startTime < 6.6) {
    m_armMotor.set(0.15);

   } else if (time - startTime < 6.7) {
    m_leftLeadMotor.set(0.2);
    m_rightLeadMotor.set(0.2);

   } else if (time - startTime < 7){
    m_intakeMotor.set(-0.5);

   } else if (time - startTime < 7.1) {
    m_intakeMotor.set(0);

   }*/ 
  }
}   


  @Override
  public void teleopInit() {
    m_leftLeadMotor.setIdleMode(IdleMode.kCoast);
    m_rightLeadMotor.setIdleMode(IdleMode.kCoast);
    m_leftFollowMotor.setIdleMode(IdleMode.kCoast);
    m_rightFollowMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX()*0.7);

    double armPower = m_operatorController.getLeftY();
    
    armPower *= 0.8;

    m_armMotor.set(armPower);

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");


    if(m_operatorController.getAButtonPressed() == true)
    {
      m_intakeMotor.set(-0.85);
    }
    else if(m_operatorController.getAButtonReleased() == true)
    {
      m_intakeMotor.stopMotor();
    }
    if(m_operatorController.getBButtonPressed() == true)
    {
      m_intakeMotor.set(1);
    }
    else if(m_operatorController.getBButtonReleased() == true)
    {
      m_intakeMotor.stopMotor();
    }
    if(m_operatorController.getYButtonPressed() == true)
    {
      m_intakeMotor.set(-0.35);
    }
    else if(m_operatorController.getYButtonReleased() == true)
    {
      m_intakeMotor.stopMotor();
    }
    if(m_operatorController.getXButtonPressed() == true)
    {
      m_intakeMotor.set(0.2);
    }
    else if(m_operatorController.getXButtonReleased() == true)
    {
      m_intakeMotor.stopMotor();
    }
    if(m_operatorController.getRightBumperPressed() == true)
    {
      m_intakeMotor.set(-0.2);
    }
    else if(m_operatorController.getRightBumperReleased() == true)
    {
      m_intakeMotor.stopMotor();
    }

    if(m_driverController.getLeftBumperPressed() == true)
    {
      if(LimelightHelpers.getFiducialID("limelight") == 8)
      {
        if(tx < 0.20)
        {
          m_robotDrive.tankDrive(0.5, -0.5);
        }
        else if(tx > 0.20)
        {
          m_robotDrive.tankDrive(0.5, -0.5);
        }
      }
    }



    }

  @Override
  public void disabledInit() {
    m_leftLeadMotor.setIdleMode(IdleMode.kBrake);
    m_rightLeadMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    double startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void testPeriodic() {
    double time = Timer.getFPGATimestamp(); 
    
    if(time - startTime < 1) {
      m_intakeMotor.set(1);
    } else {
      m_armMotor.set(0);
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  


}

