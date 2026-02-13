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
    private boolean lastAPressed = false;
    private boolean lastBPressed = false;
    private boolean lastXPressed = false;
    private boolean lastDpadRightPressed = false;

    // Intake toggle state (starts on)
    private boolean intakeOn = true;

    // Ball pusher timer
    private double ballPusherStartTime = -1;

    // Encoder ticks per revolution (REV HD Hex motor with 20:1 gearbox = 560 ticks/rev)
    private static final double TICKS_PER_REV = 537.7;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final int ROTATION_TICKS = (int)(60 * TICKS_PER_DEGREE); // 60 degrees
    private static final double ARM_POWER = 0.5;

    @Override
    public void init() {
        hw.init(hardwareMap);
        hw.lIntake.setPower(0);
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
        telemetry.addData("Robot heading", pos.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    private static final double DEADZONE = 0.2;

    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }

    private void driveControls(){
        double forward = applyDeadzone(-gamepad1.left_stick_y); //inverted y-axis
        //change once get another gamepad, the drift is increasingly high with around a 0.4-0.56 value
        double strafe = applyDeadzone(gamepad1.left_stick_x);
        double turn = applyDeadzone(gamepad1.right_stick_x);

        Pose2D pos = hw.odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) - heading);
        double sinAngle = Math.sin((Math.PI / 2) - heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = -globalForward - globalStrafe + turn;
        newWheelSpeeds[1] = -globalForward + globalStrafe - turn;
        newWheelSpeeds[2] = globalForward - globalStrafe - turn;
        newWheelSpeeds[3] = globalForward + globalStrafe + turn;

        hw.frontRight.setPower(newWheelSpeeds[0]);
        hw.frontLeft.setPower(newWheelSpeeds[1]);
        hw.backRight.setPower(newWheelSpeeds[2]);
        hw.backLeft.setPower(newWheelSpeeds[3]);

        // Debug: raw gamepad values
//        telemetry.addData("Robot XPos:", pos.getX(DistanceUnit.MM));
//        telemetry.addData("Robot YPos:", pos.getY(DistanceUnit.MM));
//        telemetry.addData("Robot Heading", heading);
//        telemetry.addData("Forward Speed:", globalForward);
//        telemetry.addData("Strafe Speed", globalStrafe);
//        telemetry.addData("Forward Speed:", globalForward);
//        telemetry.addData("Strafe Speed", globalStrafe);

        telemetry.addData("Raw LY", gamepad1.left_stick_y);
        telemetry.addData("Raw LX", gamepad1.left_stick_x);
        telemetry.addData("Raw RX", gamepad1.right_stick_x);
        // Debug: after deadzone
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        // Debug: heading and motor powers
        telemetry.addData("Heading", heading);
        telemetry.addData("FL power", newWheelSpeeds[1]);
        telemetry.addData("FR power", newWheelSpeeds[0]);
        telemetry.addData("BL power", newWheelSpeeds[3]);
        telemetry.addData("BR power", newWheelSpeeds[2]);
    }

    private void armControl(){
        // Edge detection: only trigger on button press, not while held
        boolean aPressed = gamepad1.a && !lastAPressed;
        boolean bPressed = gamepad1.b && !lastBPressed;

        // Update last state
        lastAPressed = gamepad1.a;
        lastBPressed = gamepad1.b;

        if (aPressed) {
            // Rotate 60° clockwise
            int targetPosition = hw.revolverMotor.getCurrentPosition() + ROTATION_TICKS;
            hw.revolverMotor.setTargetPosition(targetPosition);
            hw.revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.revolverMotor.setPower(ARM_POWER);
        } else if (bPressed) {
            // Rotate 60° counter-clockwise
            int targetPosition = hw.revolverMotor.getCurrentPosition() - ROTATION_TICKS;
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
        if (gamepad1.dpad_up) {
            added = added + 0.75;
        }
        if (gamepad1.dpad_down) {
            added = added - 0.75;
        }
    }
    private void geckoControl() {
        double power = (8.0 + added) * gamepad1.left_trigger;
        hw.lGecko.setPower(power);
        hw.rGecko.setPower(power);
    }

    // Power offset to counteract rubber band tension pulling servos together.
    // Positive = lIntake runs faster, Negative = rIntake runs faster.
    // Tune this value until the offset stops closing in.
    private static final double INTAKE_BAND_OFFSET = 0.05;

    private void intakeControl() {
        boolean xPressed = gamepad1.x && !lastXPressed;
        lastXPressed = gamepad1.x;

        if (xPressed) {
            intakeOn = !intakeOn;
        }

        if (intakeOn) {
            hw.lIntake.setPower(-(1.0 + INTAKE_BAND_OFFSET));
            hw.rIntake.setPower(1.0);
        } else {
            hw.lIntake.setPower(0);
            hw.rIntake.setPower(0);
        }
    }

    private void ballPusherControl() {
        boolean dpadRightPressed = gamepad1.dpad_right && !lastDpadRightPressed;
        lastDpadRightPressed = gamepad1.dpad_right;

        if (dpadRightPressed && ballPusherStartTime < 0) {
            hw.ballPusher.setPosition(0);
            ballPusherStartTime = getRuntime();
        }

        if (ballPusherStartTime >= 0 && getRuntime() - ballPusherStartTime >= 2.0) {
            hw.ballPusher.setPosition(0.5);
            ballPusherStartTime = -1;
        }
    }

}
