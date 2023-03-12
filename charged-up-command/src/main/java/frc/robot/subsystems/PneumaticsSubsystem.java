// heehee we also borrowed this from 7034

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
    public boolean isOn;
    public Compressor compressor;
    private DoubleSolenoid arm1;
    private DoubleSolenoid manipulator;

    public PneumaticsSubsystem() {
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        arm1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
        // manipulator = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
    }

    public boolean setCompressor(boolean on) {
        this.isOn = on;
        if (on) { compressor.enableDigital(); }
        else { compressor.disable(); }
        return compressor.isEnabled();
    }

    public boolean toggleCompressor() {
        return setCompressor(!compressor.isEnabled());
    }

    public void toggleArm() {
        if (arm1.get() == Value.kForward) {
            arm1.set(Value.kReverse);
        } else {
            arm1.set(Value.kForward);
        }
    }

    public void toggleManipulator() {
        if (manipulator.get() == Value.kForward) {
            manipulator.set(Value.kReverse);
        } else {
            manipulator.set(Value.kForward);
        }
    }

    public void setArm(Value value) {
        arm1.set(value);
    }

    public void setManipulator(Value value) {
        manipulator.set(value);
    }

    private boolean toBoolean(Value value) {
        return value == Value.kForward;
    }

    public void debug() {
        SmartDashboard.putBoolean("Compressor On", isOn);
        SmartDashboard.putBoolean("Arm Piston", toBoolean(arm1.get()));
        // SmartDashboard.putBoolean("Manipulator Piston", toBoolean(manipulator.get()));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        debug();
    }
    
}
