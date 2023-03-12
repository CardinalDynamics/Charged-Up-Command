package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ClackAuto extends SequentialCommandGroup {
    
    public ClackAuto(DriveSubsystem driveSubsystem) {
        addCommands(
            new RunCommand(() -> driveSubsystem.arcadeDrive(0.3, 0), driveSubsystem).withTimeout(1),
            new RunCommand(() -> driveSubsystem.arcadeDrive(-0.6, 0), driveSubsystem).withTimeout(1),
            new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(4),
            new RunCommand(() -> driveSubsystem.autoBalance(), driveSubsystem)
        );
    }
    
}
