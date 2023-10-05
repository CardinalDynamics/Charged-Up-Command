package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmLift extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier trigger1;
    private final DoubleSupplier trigger2;
    private final BooleanSupplier stop;
    private double armSpeed;
    
    public ArmLift(ArmSubsystem arm, DoubleSupplier trigger1, DoubleSupplier trigger2, BooleanSupplier stop) {
        this.arm = arm;
        this.trigger1 = trigger1;
        this.trigger2 = trigger2;
        this.stop = stop;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (trigger1.getAsDouble() > trigger2.getAsDouble()) {
            armSpeed = trigger1.getAsDouble() * 0.5;
        } else {
            armSpeed = -trigger2.getAsDouble() * 0.5;
        }
        
        if (stop.getAsBoolean()) {
            arm.setArmSpeed(0);
        } else {
        }
            arm.setArmSpeed(armSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        arm.setArmSpeed(0);
    }
}
