package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class FTChardware {
    //FTC
    //DCMOTORS
    //Wheels
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor backRight;

    //Declare Odometry computer
    public GoBildaPinpointDriver odo;
    //DCMOTORS
    //Arm
    public DcMotor revolverMotor;
    public DcMotor lGecko;
    public DcMotor rGecko;

    public Servo ballPusher;
    public CRServo rIntake;

    private VoltageSensor voltageSensor;
    private static final double NOMINAL_VOLTAGE = 12.0;


    public void init(HardwareMap hwMap) {
        //drive motors
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        //pinpoint
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        //other motors
        revolverMotor = hwMap.get(DcMotor.class, "revolverMotor");
        lGecko = hwMap.get(DcMotor.class, "lGecko");
        rGecko = hwMap.get(DcMotor.class, "rGecko");

        //Servos
        ballPusher = hwMap.get(Servo.class, "ballPusher");
        rIntake = hwMap.get(CRServo.class, "rIntake");
        rIntake.setPower(0);

        voltageSensor = hwMap.voltageSensor.iterator().next();

        //Odometry computer configuration
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        setWheelDirections();
        stopAndResetDriveEncoders();
        setArmDirection();
        setZeroPowerBehavior();
    }

    private void setWheelDirections() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        revolverMotor.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    private void stopAndResetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setArmDirection() {
        lGecko.setDirection(DcMotorSimple.Direction.REVERSE);
        rGecko.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setZeroPowerBehavior() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lGecko.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rGecko.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getVoltageCompensation() {
        return NOMINAL_VOLTAGE / voltageSensor.getVoltage();
    }

    public void setGeckoPower(double power) {
        double comp = getVoltageCompensation();
        lGecko.setPower(power * comp);
        rGecko.setPower(power * comp);
    }

    // Optional: stop all motors
    public void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        revolverMotor.setPower(0);
        lGecko.setPower(0);
        rGecko.setPower(0);
    }
}