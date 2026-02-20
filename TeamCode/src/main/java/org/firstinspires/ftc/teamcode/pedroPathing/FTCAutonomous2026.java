package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "FTCAutonomous2026")
public class FTCAutonomous2026 extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    FTChardware hw = new FTChardware();

    private static final Pose SCORE_POS = new Pose(23.18, 30.76);

    private PathChain pathToDeposit;
    private PathChain pathPush;
    private PathChain pathToScore1;
    private PathChain pathToPickup1;
    private PathChain pathIntake1;
    private PathChain pathToScore2;
    private PathChain pathToPickup2;
    private PathChain pathIntake2;
    private PathChain pathToScore3;

    private static final double INTAKE_BAND_OFFSET = 0.0;

    // Revolver constants
    private static final double TICKS_PER_REV = 448.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final double REVOLVER_POWER = 0.43;
    private static final double BALL_INTAKE_TIME = 1.0; // seconds per ball

    // Gecko/spit constants
    private static final double GECKO_POWER_1 = 0.49;
    private static final double GECKO_POWER_2 = 0.43;
    private static final double GECKO_POWER_3 = 0.41;

    private enum AutoState {
        DRIVE_TO_DEPOSIT,
        GECKO_STARTUP,
        SPIT_PUSH,
        SPIT_SELECT,
        DRIVE_PUSH,
        DRIVE_TO_SCORE_1,
        WAIT_1,
        DRIVE_TO_PICKUP_1,
        INTAKE_1,
        DRIVE_TO_SCORE_2,
        WAIT_2,
        DRIVE_TO_PICKUP_2,
        INTAKE_2,
        DRIVE_TO_SCORE_3,
        WAIT_3,
        DONE
    }

    private AutoState state = AutoState.DRIVE_TO_DEPOSIT;
    private double waitStartTime;
    private double intakeStartTime;
    private int revolverStep;
    private int spitStep;
    private double pushTimer;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(45));

    @Override
    public void init() {
        hw.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        // Start -> score position (forward 38.51", turn heading to 225°)
        pathToDeposit = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SCORE_POS))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(231))
                .build();

        // Push samples
//        pathPush = follower.pathBuilder()
//                .addPath(new BezierLine(SCORE_POS, new Pose(87.53, -17.08)))
//                .setLinearHeadingInterpolation(Math.toRadians(101), Math.toRadians(180))
//                .addPath(new BezierLine(new Pose(87.53, -17.08), new Pose(33.38, -15.10)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(new BezierLine(new Pose(33.38, -15.10), new Pose(85.56, 11.38)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Score 1
//        pathToScore1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(85.56, 11.38), SCORE_POS))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(101))
//                .build();
//
//        // Pickup 1
//        pathToPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(SCORE_POS, new Pose(83.97, 22.84)))
//                .setLinearHeadingInterpolation(Math.toRadians(101), Math.toRadians(180))
//                .build();
//
//        // Intake 1
//        pathIntake1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(83.97, 22.84), new Pose(34.17, 19.29)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(new BezierLine(new Pose(34.17, 19.29), new Pose(26.66, 35.10)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addPath(new BezierLine(new Pose(26.66, 35.10), new Pose(59.86, 20.47)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Score 2
//        pathToScore2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(59.86, 20.47), SCORE_POS))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(101))
//                .build();
//
//        // Pickup 2
//        pathToPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(SCORE_POS, new Pose(71.72, 56.05)))
//                .setLinearHeadingInterpolation(Math.toRadians(101), Math.toRadians(180))
//                .build();
//
//        // Intake 2
//        pathIntake2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(71.72, 56.05), new Pose(34.56, 55.65)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        // Score 3
//        pathToScore3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Pose(34.56, 55.65), SCORE_POS))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(101))
//                .build();

        hw.rIntake.setPower(0);
        hw.ballPusher.setPosition(0.5);

        telemetryM.debug("Autonomous initialized. Waiting for start...");
        telemetryM.update(telemetry);
    }

    private void intakeOn() {
        hw.rIntake.setPower(1.0);
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
        follower.followPath(pathToDeposit);
        state = AutoState.DRIVE_TO_DEPOSIT;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case DRIVE_TO_DEPOSIT:
                if (!follower.isBusy()) {
                    hw.setGeckoPower(GECKO_POWER_1);
                    pushTimer = getRuntime();
                    spitStep = 0;
                    state = AutoState.GECKO_STARTUP;
                }
                break;

            case GECKO_STARTUP:
                if (getRuntime() - pushTimer >= 0.8) {
                    hw.ballPusher.setPosition(0.0);
                    pushTimer = getRuntime();
                    state = AutoState.SPIT_PUSH;
                }
                break;

            case SPIT_PUSH:
                if (getRuntime() - pushTimer >= 1.0) {
                    hw.ballPusher.setPosition(0.5);
                    spitStep++;
                    if (spitStep < 3) {
                        revolverRotate(240);
                        state = AutoState.SPIT_SELECT;
                    } else {
                        hw.setGeckoPower(0);
                        state = AutoState.DONE;
                    }
                }
                break;

            case SPIT_SELECT:
                if (!hw.revolverMotor.isBusy()) {
                    double power = (spitStep == 1) ? GECKO_POWER_2 : GECKO_POWER_3;
                    hw.setGeckoPower(power);
                    hw.ballPusher.setPosition(0.0);
                    pushTimer = getRuntime();
                    state = AutoState.SPIT_PUSH;
                }
                break;

            case DRIVE_PUSH:
                if (!follower.isBusy()) {
                    follower.followPath(pathToScore1);
                    state = AutoState.DRIVE_TO_SCORE_1;
                }
                break;

            case DRIVE_TO_SCORE_1:
                if (!follower.isBusy()) {
                    waitStartTime = getRuntime();
                    state = AutoState.WAIT_1;
                }
                break;

            case WAIT_1:
                if (getRuntime() - waitStartTime >= 3.0) {
                    follower.followPath(pathToPickup1);
                    state = AutoState.DRIVE_TO_PICKUP_1;
                }
                break;

            case DRIVE_TO_PICKUP_1:
                if (!follower.isBusy()) {
                    intakeOn();
                    revolverStep = 0;
                    intakeStartTime = getRuntime();
                    follower.followPath(pathIntake1);
                    state = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1: {
                double elapsed = getRuntime() - intakeStartTime;
                if (revolverStep == 0) {
                    revolverRotate(30);
                    revolverStep = 1;
                } else if (revolverStep == 1 && elapsed >= BALL_INTAKE_TIME) {
                    revolverRotate(60);
                    revolverStep = 2;
                } else if (revolverStep == 2 && elapsed >= 2 * BALL_INTAKE_TIME) {
                    revolverRotate(60);
                    revolverStep = 3;
                } else if (revolverStep == 3 && elapsed >= 3 * BALL_INTAKE_TIME) {
                    revolverRotate(30);
                    revolverStep = 4;
                }
                if (revolverStep == 4 && !hw.revolverMotor.isBusy() && !follower.isBusy()) {
                    intakeOff();
                    follower.followPath(pathToScore2);
                    state = AutoState.DRIVE_TO_SCORE_2;
                }
                break;
            }

            case DRIVE_TO_SCORE_2:
                if (!follower.isBusy()) {
                    waitStartTime = getRuntime();
                    state = AutoState.WAIT_2;
                }
                break;

            case WAIT_2:
                if (getRuntime() - waitStartTime >= 3.0) {
                    follower.followPath(pathToPickup2);
                    state = AutoState.DRIVE_TO_PICKUP_2;
                }
                break;

            case DRIVE_TO_PICKUP_2:
                if (!follower.isBusy()) {
                    intakeOn();
                    revolverStep = 0;
                    intakeStartTime = getRuntime();
                    follower.followPath(pathIntake2);
                    state = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2: {
                double elapsed = getRuntime() - intakeStartTime;
                if (revolverStep == 0) {
                    revolverRotate(30);
                    revolverStep = 1;
                } else if (revolverStep == 1 && elapsed >= BALL_INTAKE_TIME) {
                    revolverRotate(60);
                    revolverStep = 2;
                } else if (revolverStep == 2 && elapsed >= 2 * BALL_INTAKE_TIME) {
                    revolverRotate(60);
                    revolverStep = 3;
                } else if (revolverStep == 3 && elapsed >= 3 * BALL_INTAKE_TIME) {
                    revolverRotate(30);
                    revolverStep = 4;
                }
                if (revolverStep == 4 && !hw.revolverMotor.isBusy() && !follower.isBusy()) {
                    intakeOff();
                    follower.followPath(pathToScore3);
                    state = AutoState.DRIVE_TO_SCORE_3;
                }
                break;
            }

            case DRIVE_TO_SCORE_3:
                if (!follower.isBusy()) {
                    waitStartTime = getRuntime();
                    state = AutoState.WAIT_3;
                }
                break;

            case WAIT_3:
                if (getRuntime() - waitStartTime >= 3.0) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                break;
        }

        telemetryM.debug("State: " + state);
        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        telemetryM.debug("heading: " + Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("busy: " + follower.isBusy());
        telemetryM.update(telemetry);

        Drawing.drawDebug(follower);
    }

    @Override
    public void stop() {
        hw.setGeckoPower(0);
        hw.ballPusher.setPosition(0.5);
        follower.breakFollowing();
    }
}
