// cardin please don't look at this
package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier driveY;
    private final DoubleSupplier driveX;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier joystickY, DoubleSupplier joystickX) {
        this.drive = drive;
        this.driveY = joystickY;
        this.driveX = joystickX;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = driveY.getAsDouble();
        double zRotation = driveX.getAsDouble();
        if (Math.abs(xSpeed) < Constants.OperatorConstants.yDeadband) xSpeed = 0;
        if (Math.abs(zRotation) < Constants.OperatorConstants.xDeadband) zRotation = 0;

        drive.arcadeDrive(xSpeed, zRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
