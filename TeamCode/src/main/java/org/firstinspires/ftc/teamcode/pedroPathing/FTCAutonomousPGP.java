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

@Autonomous(name = "FTCAutonomousPGP")
public class FTCAutonomousPGP extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    FTChardware hw = new FTChardware();

    private static final Pose SCORE_POS = new Pose(23.18, 30.76);
    private static final Pose PUSH_START = new Pose(0.88, 34.01);

    // TODO: Tune pile positions
    private static final Pose PILE_1_APPROACH = new Pose(-20, 40);
    private static final Pose PILE_1_END = new Pose(-50, 46);
    private static final Pose PILE_2_APPROACH = new Pose(-20, 60);
    private static final Pose PILE_2_END = new Pose(-50, 66);
    private static final Pose PILE_3_APPROACH = new Pose(-20, 80);
    private static final Pose PILE_3_END = new Pose(-50, 86);

    private static final double TICKS_PER_REV = 448.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final double REVOLVER_POWER = 0.43;
    private static final double BALL_INTAKE_TIME = 1.0;

    private static final double GECKO_POWER_1 = 0.49;
    private static final double GECKO_POWER_2 = 0.42;
    private static final double GECKO_POWER_3 = 0.45;

    // Target: P, G, P — pile arrangements are always upper=PPG, middle=PGP, lower=GPP
    private static final double[][] SPIT_ROT = {
        {0, 240, 240},    // Pre-loaded (standard, tested)
        {0, 240, 240},    // Pile 1 - upper PPG: P(slot0), G(slot2), P(slot1)
        {0, 120, 120},    // Pile 2 - middle PGP: P(slot0), G(slot1), P(slot2) - natural
        {120, 240, 240},  // Pile 3 - lower GPP: P(slot1), G(slot0), P(slot2)
    };

    private enum AutoState {
        DRIVE_TO_DEPOSIT,
        SPIT_ROTATE,
        SPIT_WAIT_ROTATE,
        SPIT_PUSH,
        DRIVE_PUSH_WAIT,
        DRIVE_TO_PILE,
        INTAKE,
        DRIVE_TO_SCORE,
        GECKO_START,
        DONE
    }

    private AutoState state;
    private int currentPile = 0;
    private int spitStep = 0;
    private int intakeStep = 0;
    private double pushTimer;
    private double intakeStartTime;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(45));

    private PathChain pathToDeposit;
    private PathChain pathPush;
    private PathChain[] pathToPile = new PathChain[3];
    private PathChain[] pathIntake = new PathChain[3];
    private PathChain[] pathToScore = new PathChain[3];

    @Override
    public void init() {
        hw.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();

        pathToDeposit = follower.pathBuilder()
                .addPath(new BezierLine(startPose, SCORE_POS))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(231))
                .build();

        pathPush = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POS, PUSH_START))
                .setLinearHeadingInterpolation(Math.toRadians(231), Math.toRadians(180))
                .build();

        pathToPile[0] = follower.pathBuilder()
                .addPath(new BezierLine(PUSH_START, PILE_1_APPROACH))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathIntake[0] = follower.pathBuilder()
                .addPath(new BezierLine(PILE_1_APPROACH, PILE_1_END))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathToScore[0] = follower.pathBuilder()
                .addPath(new BezierLine(PILE_1_END, SCORE_POS))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(231))
                .build();

        pathToPile[1] = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POS, PILE_2_APPROACH))
                .setLinearHeadingInterpolation(Math.toRadians(231), Math.toRadians(180))
                .build();
        pathIntake[1] = follower.pathBuilder()
                .addPath(new BezierLine(PILE_2_APPROACH, PILE_2_END))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathToScore[1] = follower.pathBuilder()
                .addPath(new BezierLine(PILE_2_END, SCORE_POS))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(231))
                .build();

        pathToPile[2] = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POS, PILE_3_APPROACH))
                .setLinearHeadingInterpolation(Math.toRadians(231), Math.toRadians(180))
                .build();
        pathIntake[2] = follower.pathBuilder()
                .addPath(new BezierLine(PILE_3_APPROACH, PILE_3_END))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathToScore[2] = follower.pathBuilder()
                .addPath(new BezierLine(PILE_3_END, SCORE_POS))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(231))
                .build();

        hw.rIntake.setPower(0);
        hw.ballPusher.setPosition(0.5);
        telemetryM.debug("PGP Autonomous initialized.");
        telemetryM.update(telemetry);
    }

    private void intakeOn() { hw.rIntake.setPower(1.0); }
    private void intakeOff() { hw.rIntake.setPower(0); }

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
        hw.setGeckoPower(GECKO_POWER_1);
        currentPile = 0;
        state = AutoState.DRIVE_TO_DEPOSIT;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case DRIVE_TO_DEPOSIT:
                if (!follower.isBusy()) {
                    spitStep = 0;
                    state = AutoState.SPIT_ROTATE;
                }
                break;

            case SPIT_ROTATE: {
                double rot = SPIT_ROT[currentPile][spitStep];
                if (rot > 0) {
                    revolverRotate(rot);
                    state = AutoState.SPIT_WAIT_ROTATE;
                } else {
                    hw.ballPusher.setPosition(0.0);
                    pushTimer = getRuntime();
                    state = AutoState.SPIT_PUSH;
                }
                break;
            }

            case SPIT_WAIT_ROTATE:
                if (!hw.revolverMotor.isBusy()) {
                    hw.ballPusher.setPosition(0.0);
                    pushTimer = getRuntime();
                    state = AutoState.SPIT_PUSH;
                }
                break;

            case SPIT_PUSH: {
                double pushDur = (spitStep == 0) ? 0.5 : 0.4;
                if (getRuntime() - pushTimer >= pushDur) {
                    hw.ballPusher.setPosition(0.5);
                    spitStep++;
                    if (spitStep < 3) {
                        double power = (spitStep == 1) ? GECKO_POWER_2 : GECKO_POWER_3;
                        hw.setGeckoPower(power);
                        state = AutoState.SPIT_ROTATE;
                    } else {
                        hw.setGeckoPower(0);
                        afterSpit();
                    }
                }
                break;
            }

            case DRIVE_PUSH_WAIT:
                if (pushTimer > 0 && getRuntime() - pushTimer >= 0.4) {
                    follower.followPath(pathPush);
                    pushTimer = -1;
                } else if (pushTimer <= 0 && !follower.isBusy()) {
                    follower.followPath(pathToPile[0]);
                    state = AutoState.DRIVE_TO_PILE;
                }
                break;

            case DRIVE_TO_PILE:
                if (!follower.isBusy()) {
                    intakeOn();
                    intakeStep = 0;
                    intakeStartTime = getRuntime();
                    follower.followPath(pathIntake[currentPile - 1]);
                    state = AutoState.INTAKE;
                }
                break;

            case INTAKE: {
                double elapsed = getRuntime() - intakeStartTime;
                if (intakeStep == 0) {
                    revolverRotate(120);
                    intakeStep = 1;
                } else if (intakeStep == 1 && elapsed >= BALL_INTAKE_TIME && !hw.revolverMotor.isBusy()) {
                    revolverRotate(120);
                    intakeStep = 2;
                } else if (intakeStep == 2 && elapsed >= 2 * BALL_INTAKE_TIME && !hw.revolverMotor.isBusy()) {
                    revolverRotate(120);
                    intakeStep = 3;
                } else if (intakeStep == 3 && elapsed >= 3 * BALL_INTAKE_TIME && !hw.revolverMotor.isBusy()) {
                    revolverRotate(30);
                    intakeStep = 4;
                }
                if (intakeStep == 4 && !hw.revolverMotor.isBusy() && !follower.isBusy()) {
                    intakeOff();
                    follower.followPath(pathToScore[currentPile - 1]);
                    state = AutoState.DRIVE_TO_SCORE;
                }
                break;
            }

            case DRIVE_TO_SCORE:
                if (!follower.isBusy()) {
                    hw.setGeckoPower(GECKO_POWER_1);
                    pushTimer = getRuntime();
                    state = AutoState.GECKO_START;
                }
                break;

            case GECKO_START:
                if (getRuntime() - pushTimer >= 0.8) {
                    spitStep = 0;
                    state = AutoState.SPIT_ROTATE;
                }
                break;

            case DONE:
                break;
        }

        telemetryM.debug("State: " + state);
        telemetryM.debug("Pile: " + currentPile);
        telemetryM.debug("x: " + follower.getPose().getX());
        telemetryM.debug("y: " + follower.getPose().getY());
        telemetryM.debug("heading: " + Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.update(telemetry);
        Drawing.drawDebug(follower);
    }

    private void afterSpit() {
        currentPile++;
        if (currentPile > 3) {
            state = AutoState.DONE;
        } else if (currentPile == 1) {
            pushTimer = getRuntime();
            state = AutoState.DRIVE_PUSH_WAIT;
        } else if (currentPile == 3 && getRuntime() >= 25) {
            // Not enough time for full pile 3 cycle — position near pile 3 for teleop
            follower.followPath(pathToPile[2]);
            state = AutoState.DONE;
        } else {
            follower.followPath(pathToPile[currentPile - 1]);
            state = AutoState.DRIVE_TO_PILE;
        }
    }

    @Override
    public void stop() {
        hw.setGeckoPower(0);
        hw.ballPusher.setPosition(0.5);
        intakeOff();
        follower.breakFollowing();
    }
}
