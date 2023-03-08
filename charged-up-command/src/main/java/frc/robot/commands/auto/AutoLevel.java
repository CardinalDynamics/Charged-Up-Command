package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevel extends SequentialCommandGroup {
    public AutoLevel(DriveSubsystem driveSubsystem) {
        addCommands(
            new RunCommand(() -> driveSubsystem.arcadeDrive(.6, 0), driveSubsystem).withTimeout(2),
            new RunCommand(() -> driveSubsystem.autoBalance(), driveSubsystem)
        );
    }
}
