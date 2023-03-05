package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        CameraServer.startAutomaticCapture("Arm Camera", 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
