package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Manipulator extends CommandBase {
    private final PneumaticsSubsystem pneumatics;
    private final BooleanSupplier grab;

    public Manipulator(PneumaticsSubsystem pneumatics, BooleanSupplier grab) {
        this.pneumatics = pneumatics;
        this.grab = grab;
        addRequirements(pneumatics);
    }

    @Override
    public void initialize() {
        pneumatics.setManipulator(Value.kForward);
    }

    @Override
    public void execute() {
        if (grab.getAsBoolean()) {
            pneumatics.setManipulator(Value.kReverse);
        } else {
            pneumatics.setManipulator(Value.kForward);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.setManipulator(Value.kForward);
    }
    
}
