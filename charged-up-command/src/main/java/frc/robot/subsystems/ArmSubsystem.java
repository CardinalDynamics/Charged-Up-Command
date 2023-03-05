package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor;
    private RelativeEncoder armEncoder;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        armMotor = new CANSparkMax(5, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(80);
        
        armEncoder = armMotor.getEncoder();
    }

    public void debug() {
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        debug();
    }
    
}
