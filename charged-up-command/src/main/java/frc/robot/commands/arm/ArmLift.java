package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmLift extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier speed;
    private final BooleanSupplier stop;
    
    public ArmLift(ArmSubsystem arm, DoubleSupplier speed, BooleanSupplier stop) {
        this.arm = arm;
        this.speed = speed;
        this.stop = stop;
        addRequirements(arm);
    }

    public ArmLift(ArmSubsystem arm, DoubleSupplier speedPos, DoubleSupplier speedNeg, BooleanSupplier stop) {
        this.arm = arm;
        this.speed = () -> {
            if (speedPos.getAsDouble() > 0.1) {
                return speedPos.getAsDouble();
            } else if (-speedNeg.getAsDouble() < -0.1) {
                return -speedNeg.getAsDouble();
            } else if (speedPos.getAsDouble() > 0.1 && speedNeg.getAsDouble() > 0.1) {
                return speedPos.getAsDouble();
            } else {
                return 0;
            }
        };
        this.stop = stop;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double armSpeed = speed.getAsDouble();
        
        if (stop.getAsBoolean()) {
            arm.setArmSpeed(0);
        } else {
            arm.setArmSpeed(armSpeed);
        }
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
