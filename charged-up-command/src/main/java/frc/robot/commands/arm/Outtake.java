package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Outtake extends CommandBase {

    private final IntakeSubsystem intake;
    
    public Outtake(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
