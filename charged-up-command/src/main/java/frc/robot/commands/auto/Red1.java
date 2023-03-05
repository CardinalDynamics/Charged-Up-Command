package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class Red1 extends SequentialCommandGroup {

    public Red1(DriveSubsystem drive) {
        addCommands(
            new SetOdometry(drive, "Red 1"),
            new FollowPath(drive, "Red 1", false)
        );
    }
    
}
