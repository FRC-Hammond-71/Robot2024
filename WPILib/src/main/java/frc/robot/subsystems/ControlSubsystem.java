package frc.robot.subsystems;

import java.time.Duration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AlignWithTagCommand;

/**
 * The operator-control subsystem!
 */
public class ControlSubsystem extends SubsystemBase {
    
    public Duration MaxBlockingDuration = Duration.ofSeconds(10);

    private MovementSubsystem Drive;
    private ArmSubsystem Arm;
    private Command InputBlockingCommand;

    private XboxController DriverController = new XboxController(0);
    private XboxController OperatorController = new XboxController(1);

    public ControlSubsystem(MovementSubsystem drive, ArmSubsystem arm)
    {
        super();

        this.Drive = drive;
        this.Arm = arm;
    }

    @Override
    public void periodic() 
    {
        if (this.InputBlockingCommand != null)
        {
            if (this.InputBlockingCommand.isFinished() || this.DriverController.getPOV() == 180 || !this.InputBlockingCommand.isScheduled())
            {
                // Clear the command, stop.
                CommandScheduler.getInstance().cancel(this.InputBlockingCommand);
                this.Drive.Stop();
                this.Arm.ArmMotor.stopMotor();
                this.InputBlockingCommand = null;
            }
            else return;
        }

        // Perform actions from the user!
        var targetSpeed = new ChassisSpeeds(this.DriverController.getLeftY() * 3, 0, -this.DriverController.getRightX() * 0.25);
        this.Drive.Drive(targetSpeed);

        double armPower = -this.OperatorController.getLeftY();
        this.Arm.ArmMotor.set(armPower * 0.7);
        
        if (this.OperatorController.getAButton() == true)
        {
            this.SetBlockingCommand(this.Arm.MoveTo(103)).schedule();
        }
        else if (this.DriverController.getBButton() == true)
        {
            this.SetBlockingCommand(new AlignWithTagCommand(this.Drive)).schedule();
        }

        SmartDashboard.putString("Input Blocking Command", this.InputBlockingCommand == null ? "None" : this.InputBlockingCommand.getName());
    }

    /**
     * @return Whether or not the command will be executed next cycle.
     */
    public Command SetBlockingCommand(Command command)
    {
        // Force the input-blocking command to be limited to set duration (NOTE: WILL NOT STOP EXECUTION OF ORG COMMAND)
        command = command.withTimeout(this.MaxBlockingDuration.toSeconds());

        this.InputBlockingCommand = this.InputBlockingCommand == null ? command : this.InputBlockingCommand.andThen(command);
        return this.InputBlockingCommand;
    }

    public void ClearBlockingCommand()
    {
        this.InputBlockingCommand = null;
    }
}
