package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants{
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)  //in kg
            .forwardZeroPowerAcceleration(-30.74091866335461)
            .lateralZeroPowerAcceleration(-40.52637120833944)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(0.65, 0, 0.001, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.001, 0.6, 0.05))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 0, 2, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(69.62553585983638)
            .yVelocity(59.702183040108274);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.6875)
            .strafePodX(-0.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap, String frontLeftMotor, String frontRightMotor, String backLeftMotor, String backRightMotor){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants
                        .leftFrontMotorName(frontLeftMotor)
                        .rightFrontMotorName(frontRightMotor)
                        .leftRearMotorName(backLeftMotor)
                        .rightRearMotorName(backRightMotor))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

