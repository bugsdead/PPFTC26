package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "FTCAutonomous2026")
public class
FTCAutonomous2026 extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    private PathChain path;

    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        // Build a placeholder path — customize these coordinates for your route
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(24, 0, 0)))
                .setLinearHeadingInterpolation(startPose.getHeading(), 0)
                .build();

        telemetryM.debug("Autonomous initialized. Waiting for start...");
        telemetryM.update(telemetry);
    }

    @Override
    public void init_loop() {
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        telemetryM.debug("heading: " + Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("busy: " + follower.isBusy());
        telemetryM.update(telemetry);

        Drawing.drawDebug(follower);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}
