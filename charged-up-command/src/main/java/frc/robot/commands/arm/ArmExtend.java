package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import java.util.function.BooleanSupplier;

public class ArmExtend extends CommandBase {
    private final PneumaticsSubsystem pneumatics;
    private final BooleanSupplier extend;

    public ArmExtend(PneumaticsSubsystem pneumatics, BooleanSupplier extend) {
        this.pneumatics = pneumatics;
        this.extend = extend;
        addRequirements(pneumatics);
    }

    @Override
    public void initialize() {
        pneumatics.setArm(Value.kReverse);
    }

    @Override
    public void execute() {
        if (extend.getAsBoolean()) {
            pneumatics.setArm(Value.kForward);
        } else {
            pneumatics.setArm(Value.kReverse);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setArm(Value.kReverse);
    }
}
