package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class HybridCube extends SequentialCommandGroup {

    public HybridCube(DriveSubsystem drive, PneumaticsSubsystem pneumatics, IntakeSubsystem intake) {
        addCommands(
            new RunCommand(() -> intake.toggleIntake(), intake).withTimeout(2),
            // new RunCommand(() -> intake.toggleIntake(), intake),
            new RunCommand(() -> drive.arcadeDrive(-0.4, 0), drive).withTimeout(2)
        );
    }
    
    
}
