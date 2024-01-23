package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Movement.ActualMovementSubsystem;
import frc.robot.subsystems.Movement.MovementSubsystem;

public class AlignWithTagCommand extends Command 
{
    private MovementSubsystem Drive;

    public AlignWithTagCommand(MovementSubsystem drive)
    {
        this.Drive = drive;
    }

    @Override
    public void execute() 
    {
        float kp = -0.1f;
        double min_command = Units.degreesToRadians(1.5);

        var heading_error = LimelightHelpers.getTX("limelight") * 0.5f;
        heading_error += heading_error > 0 ? min_command : -min_command;

        var turning_rate = Units.degreesToRadians(heading_error);

        this.Drive.Drive(new ChassisSpeeds(0, 0, -turning_rate));
    }    

    @Override
    public boolean isFinished() 
    {
        return Math.abs(LimelightHelpers.getTX("limelight")) < 1;
    }

    @Override
    public void end(boolean interrupted) 
    {
        this.Drive.Stop();
    }
}
