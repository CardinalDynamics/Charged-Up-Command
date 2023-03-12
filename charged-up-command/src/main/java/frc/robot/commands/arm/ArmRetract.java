package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
// import java.util.function.BooleanSupplier;

public class ArmRetract extends CommandBase {
    private final PneumaticsSubsystem pneumatics;

    public ArmRetract(PneumaticsSubsystem pneumatics) {
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pneumatics.setArm(Value.kReverse);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
