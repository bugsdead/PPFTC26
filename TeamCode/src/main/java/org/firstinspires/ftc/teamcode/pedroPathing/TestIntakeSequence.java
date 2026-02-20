package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestIntakeSequence")
public class TestIntakeSequence extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    FTChardware hw = new FTChardware();

    private static final double INTAKE_BAND_OFFSET = 0.0;

    // Revolver constants
    private static final double TICKS_PER_REV = 448.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final double REVOLVER_POWER = 0.5;

    private PathChain pathMove1;
    private PathChain pathMove2;
    private PathChain pathMove3;

    private enum TestState {
        MOVE_1,
        REVOLVER_1,
        MOVE_2,
        REVOLVER_2,
        MOVE_3,
        REVOLVER_3,
        REVOLVER_FINAL,
        DONE
    }

    private TestState state = TestState.MOVE_1;

    // Starting pose: heading 180° means robot faces -X direction
    private final Pose startPose = new Pose(87.93, -47.91, Math.toRadians(180));

    @Override
    public void init() {
        hw.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        // 3 forward moves of 5 inches each (heading 180° = -X direction)
        double startX = startPose.getX();
        double y = startPose.getY();

        pathMove1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startX, y),
                        new Pose(startX - 15, y)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pathMove2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startX - 15, y),
                        new Pose(startX - 24, y)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pathMove3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startX - 24, y),
                        new Pose(startX - 30, y)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        hw.rIntake.setPower(0);

        telemetryM.debug("TestIntakeSequence initialized.");
        telemetryM.update(telemetry);
    }

    private void intakeOn() {
        hw.rIntake.setPower(-1.0);
    }

    private void intakeOff() {
        hw.rIntake.setPower(0);
    }

    private void revolverRotate(double degrees) {
        int ticks = (int)(degrees * TICKS_PER_DEGREE);
        int target = hw.revolverMotor.getCurrentPosition() + ticks;
        hw.revolverMotor.setTargetPosition(target);
        hw.revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.revolverMotor.setPower(REVOLVER_POWER);
    }

    @Override
    public void init_loop() {
        Drawing.drawRobot(follower.getPose());
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        intakeOn();
        follower.followPath(pathMove1);
        state = TestState.MOVE_1;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case MOVE_1:
                if (!follower.isBusy()) {
                    revolverRotate(120);
                    state = TestState.REVOLVER_1;
                }
                break;

            case REVOLVER_1:
                if (!hw.revolverMotor.isBusy()) {
                    follower.followPath(pathMove2);
                    state = TestState.MOVE_2;
                }
                break;

            case MOVE_2:
                if (!follower.isBusy()) {
                    revolverRotate(120);
                    state = TestState.REVOLVER_2;
                }
                break;

            case REVOLVER_2:
                if (!hw.revolverMotor.isBusy()) {
                    follower.followPath(pathMove3);
                    state = TestState.MOVE_3;
                }
                break;

            case MOVE_3:
                if (!follower.isBusy()) {
                    revolverRotate(120);
                    state = TestState.REVOLVER_3;
                }
                break;

            case REVOLVER_3:
                if (!hw.revolverMotor.isBusy()) {
                    revolverRotate(30);
                    state = TestState.REVOLVER_FINAL;
                }
                break;

            case REVOLVER_FINAL:
                if (!hw.revolverMotor.isBusy()) {
                    intakeOff();
                    state = TestState.DONE;
                }
                break;

            case DONE:
                break;
        }

        telemetryM.debug("State: " + state);
        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        telemetryM.debug("heading: " + Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("revolver pos: " + hw.revolverMotor.getCurrentPosition());
        telemetryM.debug("revolver target: " + hw.revolverMotor.getTargetPosition());
        telemetryM.update(telemetry);

        Drawing.drawDebug(follower);
    }

    @Override
    public void stop() {
        intakeOff();
        follower.breakFollowing();
    }
}
