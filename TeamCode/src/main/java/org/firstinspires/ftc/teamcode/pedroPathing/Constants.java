package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // TODO: Weigh your robot (in kg) and update this value
    // TODO: Run the Tuning OpMode to get accurate values for all fields below

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.16047) // TODO: Set your robot's mass in kg
            .forwardZeroPowerAcceleration(-44.590512431880974)   // TODO: Run Forward Zero Power Acceleration Tuner
            .lateralZeroPowerAcceleration(-52.34671770504159)    // TODO: Run Lateral Zero Power Acceleration Tuner
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0.01, 0.015))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.005, 0.0006))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.1, 0.0005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontRight")
            .leftRearMotorName("frontLeft")
            .rightFrontMotorName("backRight")
            .rightRearMotorName("backLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.87568820743111)  // TODO: Run Forward Velocity Tuner
            .yVelocity(50.791173259104326); // TODO: Run Lateral Velocity Tuner

    // Pinpoint odometry offsets: -84.0mm = -3.307in, -168.0mm = -6.614in
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("odo")
            .forwardPodY(-3.307)
            .strafePodX(-6.614)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

