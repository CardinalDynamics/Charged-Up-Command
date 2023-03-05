package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private static final long[] legalTags = new long[]{1, 2, 3, 4, 5, 6, 7, 8};
    private static IntegerPublisher getPipePub;
    private static IntegerSubscriber getPipeSub;
    private static DoubleSubscriber tx;
    private static DoubleSubscriber ty;
    private static DoubleSubscriber tv;
    private static DoubleSubscriber ta;
    private static IntegerSubscriber tid;
    private static IntegerSubscriber tl;
    private static DoubleArraySubscriber camTran;
    private static DoubleArraySubscriber botPose;
    private static IntegerPublisher camModePub;
    private static IntegerSubscriber camModeSub;
    private static IntegerPublisher ledModePub;
    private static IntegerSubscriber ledModeSub;
    private static DoublePublisher distance;

    public VisionSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        getPipeSub = table.getIntegerTopic("getpipe").subscribe(0);

        tx = table.getDoubleTopic("tx").subscribe(0.0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        ty = table.getDoubleTopic("ty").subscribe(0.0); // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        tv = table.getDoubleTopic("tv").subscribe(0.0); // Whether the limelight has any valid targets (0 or 1)
        ta = table.getDoubleTopic("ta").subscribe(0.0); // Target Area (0% of image to 100% of image)
        tid = table.getIntegerTopic("tid").subscribe(0);
        tl = table.getIntegerTopic("tl").subscribe(999);
        camTran = table.getDoubleArrayTopic("camtran").subscribe(new double[]{});
        ledModeSub = table.getIntegerTopic("ledmode").subscribe(0); // 0 = pipeline, 1 = off, 2 = blink, 3 = on
        camModeSub = table.getIntegerTopic("cammode").subscribe(0); // 0 = vision, 1 = driver
        botPose = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[]{});

        ledModePub = table.getIntegerTopic("ledmode").publish();
        camModePub = table.getIntegerTopic("cammode").publish();
        getPipePub = table.getIntegerTopic("getpipe").publish();
        distance = table.getDoubleTopic("distance").publish();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        getTapeDistance().ifPresent((dist) -> distance.set(dist));
    }

    /**
     * Get the ID for the active pipeline
     * 
     * @return ID from 0-9
     */
    public long getActivePipeline() {
        return getPipeSub.get();
    }

    /**
     * Horizontal offset from crosshair to target.
     * 
     * @return offset from -27 to 27 degrees
     */
    public double getTargetOffsetX() {
        return tx.get();
    }

    /**
     * Vertical offset from crosshair to target.
     * 
     * @return offset from -20.5 to 20.5 degrees
     */
    public double getTargetOffsetY() {
        return ty.get();
    }

    /**
     * Whether the limelight has any valid targets (0 or 1)
     * 
     * @return true if valid target
     */
    public boolean isTargetAvailable() {
        return tv.get() == 1;
    }

    /**
     * Get area of detected target.
     *
     * @return target area from 0% to 100%.
     */
    public double getTargetArea() {
        return ta.get(0.0);
    }

    public Optional<Double> getTapeDistance() {
        double targetOffsetAngle = ty.get(-1);

        double angleToGoalRadians = Units.degreesToRadians(targetOffsetAngle); // come back and add mount angle
        if (isTargetAvailable()) {
            return Optional.of(Vision.goalHeightInches - Vision.cameraHeightInches / Math.tan(angleToGoalRadians));
        }
        return Optional.empty();
    }

    /**
     * Get ID of detected AprilTag
     * 
     * @return ID from 1-8
     */
    public long getTagID() {
        long id = tid.get(-1);
        for (long i : legalTags) {
            if (id == i) {
                return id;
            }
        }
        return -1;
    }

    /**
     * Get Pipeline latency + 11ms for Image Capture
     * 
     * @return latency in ms
     */
    public long getLatency() {
        return tl.get(998) + 11;
    }

    /**
     * Get Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * 
     * @return Transform from Camera to Target
     */
    public Transform3d getCamTransform3d() {
        double[] camTransform = camTran.get(new double[0]);
        return new Transform3d(
            new Translation3d(camTransform[0], camTransform[1], camTransform[2]),
            new Rotation3d(camTransform[3], camTransform[4], camTransform[5])
        );
    }

    /**
     * Get Camera transform in target space of primary apriltag or solvepnp target. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * 
     * @return Transform from Camera to Target
     */
    public Transform2d getCamTransform2d() {
        double[] camTransform = camTran.get(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        if (0 != camTransform.length) {
            return new Transform2d(
                    new Translation2d(camTransform[0], camTransform[1]),
                    new Rotation3d(Units.degreesToRadians(camTransform[3]),
                    Units.degreesToRadians(camTransform[4]),
                    Units.degreesToRadians(camTransform[5])).toRotation2d());
        }
        return new Transform2d();
    }

    /**
     * Get Robot transform in field-space. NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     * LimeLight has its own dict for apriltag poses
     * 
     * @return Pose3d of Robot
     */
    public Pose3d getBotPose() {
        double[] poseValues = botPose.get();
        if (poseValues.length != 0) {
            return new Pose3d(
                new Translation3d(poseValues[0], poseValues[1], poseValues[2]),
                new Rotation3d(Units.degreesToRadians(poseValues[3]), Units.degreesToRadians(poseValues[4]), Units.degreesToRadians(poseValues[5]))
            );
        }
        return new Pose3d();
    }

    /**
     * Method to set the green light's status
     * 
     * @param mode 0 = pipeline, 1 = off, 2 = blink, 3 = on
     */
    public void setLEDMode(int mode) {
        ledModePub.set(mode);
    }

    /** 
     * Methods for external classes to change green light's status
     */
    public void turnOnLED() {
        setLEDMode(3);
    }

    public void turnOffLED() {
        setLEDMode(1);
    }

    public void blinkLED() {
        setLEDMode(2);
    }

    public void setPipelineLED() {
        setLEDMode(0);
    }

    /**
     * Method to set the camera's status
     * 
     * @param mode 0 = vision, 1 = driver
     */
    public void setCamMode(int mode) {
        camModePub.set(mode);
    }

    /**
     * Method to set  video feed in driver mode.
     * Turns off LED and sets cam mode to driver.
     */
    public void setModeDriver() {
        setLEDMode(1);
        setCamMode(1);
    }

    /**
     * Method to set video feed in vision mode.
     * Turns on green light and switches camera mode to vision.
     */
    public void setModeVision() {
        this.setLEDMode(3);
        this.setCamMode(0);
    }

    /**
     * Methods to tell whether the limelight is in driver or vision mode.
     * Driver mode will consist of the LEDs being off and the camera being in color.
     * Vision mode will consist of the LEDs being on and the camera being in black and white.
     */
    private boolean isModeDriver() {
        return ledModeSub.get(0) == 1 && camModeSub.get(0) == 1;
    }

    private boolean isModeVision() {
        return ledModeSub.get(0) == 3 && camModeSub.get(0) == 0;
    }

    /**
    * Method to toggle the type of video feed.
    */
    public void toggleMode() {
        if (this.isModeDriver()) {
            this.setModeVision();
        } else if (this.isModeVision()) {
            this.setModeDriver();
        } else {
            this.blinkLED();
        }
    }

    public void debug() {
        SmartDashboard.putNumber("Target Area", getTargetArea());
        SmartDashboard.putNumber("Offset Y", getTargetOffsetY());
        SmartDashboard.putNumber("Offset X", getTargetOffsetX());
        SmartDashboard.putNumber("TargetID", getTagID());
        SmartDashboard.putNumber("Latency", getLatency());
    }
}
