package frc.robot.subsystems;

import java.time.Duration;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AlightWithSpeakerCommand;
import frc.robot.commands.AlignWithTagCommand;
import frc.robot.subsystems.Movement.*;

/**
 * The operator-control subsystem!
 */
public class ControlSubsystem extends SubsystemBase {

    public Duration MaxBlockingDuration = Duration.ofSeconds(30);

    private DriveSubsystem Drive;
    private ArmSubsystem Arm;
    private Command InputBlockingCommand;

    private XboxController DriverController = new XboxController(0);
    private XboxController OperatorController = new XboxController(1);

    public ControlSubsystem(DriveSubsystem drive, ArmSubsystem arm) {
        super();

        this.Drive = drive;
        this.Arm = arm;

        Shuffleboard.getTab("Drive").addString("Input Blocking Command",
                () -> this.InputBlockingCommand == null ? "None" : this.InputBlockingCommand.getName());
    }

    @Override
    public void periodic() {
        if (this.InputBlockingCommand != null) {
            if (this.InputBlockingCommand.isFinished()
                    || this.DriverController.getPOV() == 180
                    || !this.InputBlockingCommand.isScheduled()) {

                // Clear the command, stop.
                CommandScheduler.getInstance().cancel(this.InputBlockingCommand);
                this.Drive.Stop();
                this.Arm.ArmMotor.stopMotor();
                this.InputBlockingCommand = null;
            } else return;
        }

        if (DriverController.getXButtonPressed() == true) {

            System.out.println("Wooo!");
            this.SetBlockingCommand(new AlightWithSpeakerCommand(this.Drive)).schedule();

        }

        if (OperatorController.getAButton()) {
            this.Arm.IntakeMotor.set(-0.85);
        }
        else if (OperatorController.getBButton()) {
            this.Arm.IntakeMotor.set(1);
        }
        else if (OperatorController.getYButton()) {
            this.Arm.IntakeMotor.set(-0.35);
        }
        else if (OperatorController.getXButton()) {
            this.Arm.IntakeMotor.set(0.2);
        } else {
            this.Arm.IntakeMotor.stopMotor();
        }

        double armPower = -this.OperatorController.getLeftY();
        this.Arm.ArmMotor.set(armPower * 0.8);

        var forward = this.DriverController.getLeftY();
        if (Math.abs(forward) < 0.1) {
            forward = 0;
        }

        var targetSpeed = new ChassisSpeeds(forward * 4, 0,
                this.DriverController.getRightX() * Units.degreesToRadians(360));

        this.Drive.Drive(targetSpeed);
    }

    /**
     * @return Whether or not the command will be executed next cycle.
     */
    public Command SetBlockingCommand(Command command) {
        // Force the input-blocking command to be limited to set duration (NOTE: WILL
        // NOT STOP EXECUTION OF ORG COMMAND)
        // command = command.withTimeout(this.MaxBlockingDuration.toSeconds());

        this.InputBlockingCommand = this.InputBlockingCommand == null ? command
                : this.InputBlockingCommand.andThen(command);
        return this.InputBlockingCommand;
    }

    public void ClearBlockingCommand() {
        if (this.InputBlockingCommand != null) {
            this.InputBlockingCommand.cancel();
            this.InputBlockingCommand = null;
        }
    }
}
