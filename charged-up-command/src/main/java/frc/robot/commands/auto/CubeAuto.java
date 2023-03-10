package frc.robot.commands.auto;

import frc.robot.commands.arm.*;
import frc.robot.commands.auto.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.*;

public class CubeAuto extends SequentialCommandGroup {
    public CubeAuto(DriveSubsystem drive, PneumaticsSubsystem pneumatics, IntakeSubsystem intake) {
        addCommands(
            new DriveCommand(drive, () -> -0.3, () -> 0).withTimeout(1),
            new ArmExtend(pneumatics),
            new Outtake(intake).withTimeout(1.5),
            new ArmRetract(pneumatics),
            new RunCommand(() -> drive.arcadeDrive(-0.5, 0), drive).withTimeout(2)
        );
    }
}
