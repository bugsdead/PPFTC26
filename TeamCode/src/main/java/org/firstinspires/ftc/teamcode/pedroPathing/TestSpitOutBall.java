package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestSpitOutBall")
public class TestSpitOutBall extends OpMode {
    private TelemetryManager telemetryM;
    FTChardware hw = new FTChardware();

    private static final double TICKS_PER_REV = 448.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final double REVOLVER_POWER = 0.43;
    private static final double GECKO_POWER_1 = 0.49;  // 1st ball - needs more
    private static final double GECKO_POWER_2 = 0.43;  // 2nd ball - perfect
    private static final double GECKO_POWER_3 = 0.41;  // 3rd ball - needs less

    private enum SpitState {
        GECKO_STARTUP,
        SELECT,
        PUSH,
        WAIT,
        DONE
    }

    private SpitState state = SpitState.SELECT;
    private int spitStep = 0;
    private double pushTimer;

    private void revolverRotate(double degrees) {
        int ticks = (int)(degrees * TICKS_PER_DEGREE);
        int target = hw.revolverMotor.getCurrentPosition() + ticks;
        hw.revolverMotor.setTargetPosition(target);
        hw.revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.revolverMotor.setPower(REVOLVER_POWER);
    }

    @Override
    public void init() {
        hw.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        hw.ballPusher.setPosition(0.5);

        telemetryM.debug("TestSpitOutBall initialized.");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        // Turn on gecko wheels first, let them spin up
        hw.setGeckoPower(GECKO_POWER_1);
        pushTimer = getRuntime();
        spitStep = 0;
        state = SpitState.GECKO_STARTUP;
    }

    @Override
    public void loop() {
        switch (state) {
            case GECKO_STARTUP:
                if (getRuntime() - pushTimer >= 0.8) {
                    hw.ballPusher.setPosition(0.0);
                    pushTimer = getRuntime();
                    state = SpitState.PUSH;
                }
                break;

            case PUSH:
                if (getRuntime() - pushTimer >= 1.0) {
                    hw.ballPusher.setPosition(0.5);
                    spitStep++;
                    if (spitStep < 3) {
                        revolverRotate(240);
                        state = SpitState.SELECT;
                    } else {
                        hw.setGeckoPower(0);
                        state = SpitState.DONE;
                    }
                }
                break;

            case SELECT:
                if (!hw.revolverMotor.isBusy()) {
                    double power = (spitStep == 1) ? GECKO_POWER_2 : GECKO_POWER_3;
                    hw.setGeckoPower(power);
                    hw.ballPusher.setPosition(0.0);
                    pushTimer = getRuntime();
                    state = SpitState.PUSH;
                }
                break;

            case DONE:
                break;
        }

        telemetryM.debug("State: " + state);
        telemetryM.debug("Spit step: " + spitStep + "/3");
        telemetryM.debug("Revolver pos: " + hw.revolverMotor.getCurrentPosition());
        telemetryM.debug("Revolver target: " + hw.revolverMotor.getTargetPosition());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        hw.setGeckoPower(0);
        hw.ballPusher.setPosition(0.5);
    }
}
