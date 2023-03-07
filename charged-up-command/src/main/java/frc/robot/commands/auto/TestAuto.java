package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class TestAuto extends SequentialCommandGroup {
    
    public TestAuto(DriveSubsystem drive) {
        addCommands(
            new SetOdometry(drive, "Test Auto"),
            new FollowPath(drive, "Test Auto", false).getRamseteCommand()
        );
    }
    
}
