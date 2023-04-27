package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PDHSubsystem extends SubsystemBase {
    private final PowerDistribution pdh;

    public PDHSubsystem() {
        pdh = new PowerDistribution(1, ModuleType.kRev);
        clearStickyFaults();
    }

    public double getVoltage() {
        return pdh.getVoltage();
    }

    public double getCurrent(int channel) {
        return pdh.getCurrent(channel);
    }

    public double getTotalCurrent() {
        return pdh.getTotalCurrent();
    }

    public double getTotalPower() {
        return pdh.getTotalPower();
    }

    public double getTotalEnergy() {
        return pdh.getTotalEnergy();
    }

    public void resetTotalEnergy() {
        pdh.resetTotalEnergy();
    }

    public void clearStickyFaults() {
        pdh.clearStickyFaults();
    }

    public void toggleSwitchableChannel() {
        if (pdh.getSwitchableChannel() == false) {
            pdh.setSwitchableChannel(true);
        } else {
            pdh.setSwitchableChannel(false);
        }
    }

    public void periodic() {
        // This method will be called once per scheduler run
        debug();
    }

    public void debug() {
        SmartDashboard.putNumber("PDH Voltage", getVoltage());
        SmartDashboard.putNumber("PDH Total Current", getTotalCurrent());
        // SmartDashboard.putNumber("PDH Total Power", getTotalPower());
        // SmartDashboard.putNumber("PDH Total Energy", getTotalEnergy());
    }
}
