package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.units.Distance;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.2)
            .forwardZeroPowerAcceleration(-39.40622042)
            .lateralZeroPowerAcceleration(-86.65760804)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.006, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.09,0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("motor_c2")
            .rightRearMotorName("motor_c3")
            .leftRearMotorName("motor_c0")
            .leftFrontMotorName("motor_c1")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(74.4143976908)
            .yVelocity(50.5478407734)
            ;

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("motor_c3")
            .strafeEncoder_HardwareMapName("motor_c0")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            )
            .forwardPodY(Distance.fromMm(-129.2975).inIn)
            .strafePodX(Distance.fromMm(-144.7725).inIn)
            .forwardEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.00305971)
            .strafeTicksToInches(0.00307349)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
