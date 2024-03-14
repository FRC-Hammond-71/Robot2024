package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.RobotSubsystem;
import frc.robot.Constants;

public class LocalizedDriveSubsystem extends RobotSubsystem<frc.robot.Robot>
{
    private CANSparkMax LeftLeadMotor, RightLeadMotor, LeftFollowMotor, RightFollowMotor;
    public DifferentialDriveKinematics Kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TrackWidth);
    private SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(0.10158, 2, 0.53799);
    private MedianFilter InputFilter = new MedianFilter(3);

    public LocalizedDriveSubsystem(frc.robot.Robot robot)
    {
        super(robot);
    }
}
