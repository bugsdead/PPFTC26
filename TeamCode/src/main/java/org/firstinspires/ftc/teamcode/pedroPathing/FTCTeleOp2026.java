package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name= "FTCTeleOp2026")
public class FTCTeleOp2026 extends OpMode{
    //all the hardware and initialization
    FTChardware hw = new FTChardware();

    // Edge detection for button presses
    private boolean lastrbPressed = false;
    private boolean lastlbPressed = false;
    private boolean lastaPressed = false;
    private boolean lastXPressed = false;
    private boolean lastRTriggerPressed = false;
    private boolean lastDpadUpPressed = false;
    private boolean lastDpadDownPressed = false;

    // Intake toggle state (starts on)
    private boolean intakeOn = true;

    // Ball pusher timer
    private double ballPusherStartTime = -1;

    // Encoder ticks per revolution (REV HD Hex motor with 20:1 gearbox = 560 ticks/rev)
    private static final double TICKS_PER_REV =  448.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final int ROTATION_TICKS = (int)(60 * TICKS_PER_DEGREE); // 60 degrees
    private static final double ARM_POWER = 0.5;

    @Override
    public void init() {
        hw.init(hardwareMap);
        hw.rIntake.setPower(0);
        hw.ballPusher.setPosition(0.5);
        telemetry.addLine("Robot is initialized");
        telemetry.update();

        //Odometry starting position CHANGE position later
        hw.odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM,-923.925,160.49, AngleUnit.RADIANS, 0);
        hw.odo.setPosition(startingPosition);

        telemetry.addData("Status","initialized");
        telemetry.addData("X-Offset", hw.odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y-offset", hw.odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", hw.odo.getDeviceVersion());
        telemetry.addData("Device Scalar", hw.odo.getYawScalar());
        telemetry.update();

    }

    @Override
    public void loop() {
        hw.odo.update();
        driveControls();
        armControl();
        dpadSetSpeed();
        geckoControl();
        intakeControl();
        ballPusherControl();

        Pose2D pos = hw.odo.getPosition();
        telemetry.addData("Robot X", hw.odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y", hw.odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot heading", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    private static final double DEADZONE = 0.2;

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }

    private void driveControls(){
//        double forward = applyDeadzone(-gamepad1.left_stick_y); //inverted y-axis
//        //change once get another gamepad, the drift is increasingly high with around a 0.4-0.56 value
//        double strafe = applyDeadzone(gamepad1.left_stick_x);
//        double turn = applyDeadzone(gamepad1.right_stick_x);

        double forward = Math.abs(gamepad1.left_stick_y) > DEADZONE ? -gamepad1.left_stick_y : 0;
        double strafing = Math.abs(gamepad1.left_stick_x) > DEADZONE ? gamepad1.left_stick_x : 0;
        double turn = Math.abs(gamepad1.right_stick_x) > DEADZONE ? gamepad1.right_stick_x : 0;

        // Mecanum Drive Power Calculations
        hw.frontLeft.setPower(-forward + strafing - turn);
        hw.frontRight.setPower(forward + strafing + turn);
        hw.backLeft.setPower(-forward - strafing + turn);
        hw.backRight.setPower(forward - strafing - turn);


//        Pose2D pos = hw.odo.getPosition();
//        double heading = pos.getHeading(AngleUnit.RADIANS);
//
//        double cosAngle = Math.cos((Math.PI / 2) - heading);
//        double sinAngle = Math.sin((Math.PI / 2) - heading);
//
//        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
//        double globalForward = forward * cosAngle + strafe * sinAngle;
//
//        double[] newWheelSpeeds = new double[4];
//
//        newWheelSpeeds[0] = -globalForward - globalStrafe + turn;
//        newWheelSpeeds[1] = -globalForward + globalStrafe - turn;
//        newWheelSpeeds[2] = globalForward - globalStrafe - turn;
//        newWheelSpeeds[3] = globalForward + globalStrafe + turn;
//
//        hw.frontRight.setPower(newWheelSpeeds[0]);
//        hw.frontLeft.setPower(newWheelSpeeds[1]);
//        hw.backRight.setPower(newWheelSpeeds[2]);
//        hw.backLeft.setPower(newWheelSpeeds[3]);

        // Debug: raw gamepad values
//        telemetry.addData("Robot XPos:", pos.getX(DistanceUnit.MM));
//        telemetry.addData("Robot YPos:", pos.getY(DistanceUnit.MM));
//        telemetry.addData("Robot Heading", heading);
//        telemetry.addData("Forward Speed:", globalForward);
//        telemetry.addData("Strafe Speed", globalStrafe);
//        telemetry.addData("Forward Speed:", globalForward);
//        telemetry.addData("Strafe Speed", globalStrafe);

//        telemetry.addData("Raw LY", gamepad1.left_stick_y);
//        telemetry.addData("Raw LX", gamepad1.left_stick_x);
//        telemetry.addData("Raw RX", gamepad1.right_stick_x);
        // Debug: after deadzone
//        telemetry.addData("Forward", forward);
//        telemetry.addData("Strafe", strafe);
//        telemetry.addData("Turn", turn);
//        // Debug: heading and motor powers
//        telemetry.addData("Heading", heading);
//        telemetry.addData("FL power", newWheelSpeeds[1]);
//        telemetry.addData("FR power", newWheelSpeeds[0]);
//        telemetry.addData("BL power", newWheelSpeeds[3]);
//        telemetry.addData("BR power", newWheelSpeeds[2]);
    }

    private void armControl(){
        // Edge detection: only trigger on button press, not while held
        boolean rbPressed = gamepad1.right_bumper && !lastrbPressed;
        boolean lbPressed = gamepad1.left_bumper && !lastlbPressed;
        boolean aPressed = gamepad1.a && !lastaPressed;

        // Update last state
        lastrbPressed = gamepad1.right_bumper;
        lastlbPressed = gamepad1.left_bumper;
        lastaPressed = gamepad1.a;

        if (rbPressed) {
            // Rotate 60° clockwise
            int targetPosition = hw.revolverMotor.getCurrentPosition() + (2 * ROTATION_TICKS);
            hw.revolverMotor.setTargetPosition(targetPosition);
            hw.revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.revolverMotor.setPower(ARM_POWER);
        } else if (lbPressed) {
            // Rotate 60° counter-clockwise
            int targetPosition = hw.revolverMotor.getCurrentPosition() - (2 * ROTATION_TICKS);
            hw.revolverMotor.setTargetPosition(targetPosition);
            hw.revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.revolverMotor.setPower(ARM_POWER);
        } else if (aPressed) {
            //Rotate 30° clockwise
            int targetPosition = hw.revolverMotor.getCurrentPosition() + (ROTATION_TICKS);
            hw.revolverMotor.setTargetPosition(targetPosition);
            hw.revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.revolverMotor.setPower(ARM_POWER);
        }

        // Telemetry for debugging
        telemetry.addData("Arm Position", hw.revolverMotor.getCurrentPosition());
        telemetry.addData("Arm Target", hw.revolverMotor.getTargetPosition());
    }

    public double added = 0;
    private void dpadSetSpeed() {
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUpPressed;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDownPressed;
        lastDpadUpPressed = gamepad1.dpad_up;
        lastDpadDownPressed = gamepad1.dpad_down;

        if (dpadUpPressed) {
            added += 0.15;
        }
        if (dpadDownPressed) {
            added -= 0.15;
        }
    }
    private void geckoControl() {
        double power = (0.6 + added) * gamepad1.left_trigger;
        hw.lGecko.setPower(power);
        hw.rGecko.setPower(power);
        telemetry.addData("Added: ", added);
        telemetry.addData("Power: ", power);
    }

    private void intakeControl() {
        boolean xPressed = gamepad1.x && !lastXPressed;
        lastXPressed = gamepad1.x;

        if (xPressed) {
            intakeOn = !intakeOn;
        }

        if (intakeOn) {
            hw.rIntake.setPower(0.5);
        } else {
            hw.rIntake.setPower(0);
        }
    }

    private void ballPusherControl() {
        boolean rTriggerPressed = (gamepad1.right_trigger > 0.5) && !lastRTriggerPressed;
        lastRTriggerPressed = (gamepad1.right_trigger > 0.5);

        if (rTriggerPressed && ballPusherStartTime < 0) {
            hw.ballPusher.setPosition(0);
            ballPusherStartTime = getRuntime();
        }

        if (ballPusherStartTime >= 0 && getRuntime() - ballPusherStartTime >= 1.0) {
            hw.ballPusher.setPosition(0.5);
            ballPusherStartTime = -1;
        }
    }

}
