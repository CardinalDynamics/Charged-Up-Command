package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFullSend extends CommandBase {
    private final IntakeSubsystem intake;
    
    public IntakeFullSend(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        intake = intakeSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(-1);
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
