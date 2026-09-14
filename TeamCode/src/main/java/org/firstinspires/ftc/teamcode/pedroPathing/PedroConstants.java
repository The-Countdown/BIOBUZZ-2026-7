package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12) //kg
            .forwardZeroPowerAcceleration(-78.97389997560546)
            .lateralZeroPowerAcceleration(-74.45025729252512)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.00002, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(0.75,0,0.06,0.00002))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.7,0,0.16,0.6, 0.1));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")//rev
            .rightRearMotorName("rightBack")//forw
            .leftRearMotorName("leftBack")//rev
            .leftFrontMotorName("leftFront") //rev
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.08652526374877)
            .yVelocity(61.17243440883366);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-119)
            .strafePodX(3)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new
            PathConstraints(0.99,
            100,
            1.6,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();


    }
}
