package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class TheLimeLight {

    public Limelight3A limelight;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    //hardwareMap

    /**
     * Constructor for the LimelightVision subsystem.
     * @param hwMap The hardware map of the robot.
     * @param telemetry The telemetry object for driver station feedback (optional, can be null).
     */
    public TheLimeLight(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hwMap;
//        limelight = hwMap.get(Limelight3A.class, hwName);
//
//        // Optional: set the poll rate (how often data is requested)
//        limelight.setPollRateHz(50); // 50 times per second is a good rate
//        limelight.start(); // Start the vision client
    }

    public TheLimeLight(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(String hwName) {
        // The device name "limelight" must match the name configured in the Rev Control Hub
        limelight = hardwareMap.get(Limelight3A.class, hwName);

        limelight.pipelineSwitch(8);

        // Optional: set the poll rate (how often data is requested)
        limelight.setPollRateHz(50); // 50 times per second is a good rate
        limelight.start(); // Start the vision client
    }


    /**
     * Gets the latest result from the Limelight.
     * @return The latest LLResult object.
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Retrieves a list of detected AprilTag IDs.
     * @return A list of integer AprilTag IDs.
     */
    public List<Integer> getDetectedAprilTagIDs() {
        //List<FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        LLResult result = getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        List<Integer> apriltagIDs = new ArrayList<>();
        if (fiducials != null) {
            //for (FiducialResult fiducial : fiducials) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                apriltagIDs.add(fiducial.getFiducialId()); // The ID number of the fiducial
            }
        }
        return apriltagIDs;
    }

    /**
     * Gets the horizontal offset (tx) from the target center in degrees.
     * @return The tx value.
     */
    public double getTargetXOffset() {
        return limelight.getLatestResult().getTx();
    }

    /**
     * Gets the vertical offset (ty) from the target center in degrees.
     * @return The ty value.
     */
    public double getTargetYOffset() {
        return limelight.getLatestResult().getTy();
    }

    /**
     * Gets the target area (ta) as a percentage of the image.
     * @return The ta value.
     */
    public double getTargetArea() {
        return limelight.getLatestResult().getTa();
    }

    /**
     * Updates telemetry with current Limelight data.
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            //telemetry.addData("Limelight Target Found", limelight.getLatestResult().getTargetFound());
            telemetry.addData("Limelight2 X Offset", getTargetXOffset());
            telemetry.addData("Limelight2 Area", getTargetArea());
            List<Integer> ids = getDetectedAprilTagIDs();
            if (!ids.isEmpty()) {
                telemetry.addData("Detected Tag IDs", ids.toString());
            }
        }
    }

    public double getDistance(){
        // Get Limelight results
        double MOUNT_ANGLE = 25.2;
        double CAMERA_HEIGHT = 14.5;
        double TARGET_HEIGHT = 33.0;
        double distance = 0;
        //double angleA1 = 26.9;
        //double angleA2 = result.getTy();
        //double cameraHeight_H1 = 13.9;
        //double aprilTagHeight_H2 = 33.5;
        //double angleToRadian =  (angleA1+angleA2)*(Math.PI / 180);
        //double distanceCal = (aprilTagHeight_H2 - cameraHeight_H1) / Math.tan(angleToRadian);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();

            // 1. Calculate distance (example in inches)
            double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE + ty);
            distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);

            // 2. Get required speed from a lookup function or formula
            //double targetVelocity = calculateVelocityFromDistance(distance);

            // 3. Set motor velocity (ticks per second)
            //motor.setVelocity(targetVelocity);
        }
        return distance;
    }


    public void stop(){
        limelight.shutdown();
    }
    // You can add more methods here to control Limelight settings like pipeline, LED mode, etc.
}