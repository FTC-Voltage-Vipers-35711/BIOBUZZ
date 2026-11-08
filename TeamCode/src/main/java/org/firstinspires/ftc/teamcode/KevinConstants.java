package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants =
            new FollowerConstants();

    public static MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1)

                    .rightFrontMotorName("front_right_motor")
                    .rightRearMotorName("back_right_motor")
                    .leftRearMotorName("back_left_motor")
                    .leftFrontMotorName("front_left_motor")

                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static ThreeWheelConstants localizerConstants =
            new ThreeWheelConstants()
                    .forwardTicksToInches(0.001979)
                    .strafeTicksToInches(0.001979)
                    .turnTicksToInches(0.001979)

                    .leftPodY(5)
                    .rightPodY(-5)
                    .strafePodX(-5)

                    .leftEncoder_HardwareMapName("front_left_motor")
                    .rightEncoder_HardwareMapName("front_right_motor")
                    .strafeEncoder_HardwareMapName("back_left_motor")

                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.FORWARD)
                    .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {

        Follower follower = new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
        hardwareMap.get(DcMotor.class, "front_left_motor")
                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardwareMap.get(DcMotor.class, "back_left_motor")
                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardwareMap.get(DcMotor.class, "front_right_motor")
                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardwareMap.get(DcMotor.class, "back_right_motor")
                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return follower;



    }
}
