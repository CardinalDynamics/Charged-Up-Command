package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.arm.*;

public class HybridCube extends SequentialCommandGroup {

    public HybridCube(DriveSubsystem drive, ArmSubsystem arm, PneumaticsSubsystem pneumatics) {
        addCommands(
            new SetOdometry(drive, "Hybrid Cube"),
            new FollowPath(drive, "Hybrid Cube", false),
            new ArmLift(arm, () -> 0.5, () -> false),
            new ArmExtend(pneumatics, () -> true),
            new Manipulator(pneumatics, () -> true),
            new ArmLift(arm, () -> -0.5, () -> false),
            new ArmExtend(pneumatics, () -> false),
            new Manipulator(pneumatics, () -> false)
        );
    }
    
    
}
