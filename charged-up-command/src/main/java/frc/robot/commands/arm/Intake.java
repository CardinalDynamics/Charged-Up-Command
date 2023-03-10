package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Intake extends CommandBase {

    private final IntakeSubsystem intake;
    
    public Intake(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
