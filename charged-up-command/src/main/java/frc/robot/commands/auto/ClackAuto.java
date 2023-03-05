package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class ClackAuto extends SequentialCommandGroup {
    
    public ClackAuto(DriveSubsystem drive) {
        addCommands(
            new SetOdometry(drive, "Clack Auto"),
            new FollowPath(drive, "Clack Auto", false)
        );
    }
    
}
