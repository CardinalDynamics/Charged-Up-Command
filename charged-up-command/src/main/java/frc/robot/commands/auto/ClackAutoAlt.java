package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.*;

public class ClackAutoAlt extends SequentialCommandGroup {
    public ClackAutoAlt(DriveSubsystem driveSubsystem) {
        addCommands(
            new RunCommand(() -> driveSubsystem.arcadeDrive(0.3, 0), driveSubsystem).withTimeout(1),
            new RunCommand(() -> driveSubsystem.arcadeDrive(-0.6, 0), driveSubsystem).withTimeout(1),
            new RunCommand(() -> driveSubsystem.arcadeDrive(0.5, 0), driveSubsystem).withTimeout(2.5)
        );
    }
}
