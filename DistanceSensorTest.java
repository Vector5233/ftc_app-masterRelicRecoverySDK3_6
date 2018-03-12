package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by CCA on 2/6/2018.
 */

@TeleOp(name="DistanceSensorTest", group = "myGroup")
public class DistanceSensorTest extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight, liftMotor;
    ColorSensor colorSensor;
    float red, green, blue;
    Servo topLeftGrab, topRightGrab, bottomLeftGrab, bottomRightGrab;
    Servo jewelKnocker, jewelRaiser;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor rangeRight, rangeLeft;

    final double RIGHTGrab_COMPLETEOPEN = 0.8;
    final double RIGHTGrab_CLOSE = 0.30; //used to be 0.33
    final double LEFTGrab_COMPLETEOPEN = 0.2;
    final double LEFTGrab_CLOSE = 0.7; //used to be 0.67
    final double RIGHTGrab_OPEN = 0.5;
    final double LEFTGrab_OPEN = 0.5;
    final double RIGHTBottom_CLOSE = 0.4; //used to be 0.27
    final double LEFTBottom_CLOSE = 0.6; //used to be 0.73
    final double RIGHTBottom_OPEN= 0.6;
    final double LEFTBottom_OPEN= 0.4;
    //Bottom grabber values are reverse (put right in left, left in right)

    // Right, left, and center are facing the back of the bot
    final double JEWEL_UP = 0.94;
    final double JEWEL_DOWN = 0.38;
    final double JEWEL_RIGHT = 0.25;
    final double JEWEL_CENTER = 0.15;
    final double JEWEL_LEFT = 0.05;
    final double JEWEL_RETRY = 0.12;
    final double JEWEL_R_REST = 0.7;
    final double JEWEL_K_REST = 0.7;

    public void init() {
        rangeRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");
        rangeLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");

        jewelKnocker = hardwareMap.servo.get("jewel");
        jewelRaiser = hardwareMap.servo.get("raise");
        jewelRaiser.setPosition(JEWEL_UP);
        jewelKnocker.setPosition(JEWEL_CENTER);

        topRightGrab = hardwareMap.servo.get("topRightGrab");
        topLeftGrab = hardwareMap.servo.get("topLeftGrab");
        bottomLeftGrab = hardwareMap.servo.get("bottomLeftGrab");
        bottomRightGrab = hardwareMap.servo.get("bottomRightGrab");

        topRightGrab.setPosition(RIGHTGrab_COMPLETEOPEN);
        topLeftGrab.setPosition(LEFTGrab_COMPLETEOPEN);
        bottomRightGrab.setPosition(LEFTGrab_COMPLETEOPEN);
        bottomLeftGrab.setPosition(RIGHTGrab_COMPLETEOPEN);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop() {
        telemetry.addData("Right Distance", "%.2f in", rangeRight.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", "%.2f in", rangeLeft.getDistance(DistanceUnit.INCH));
            telemetry.update();

        frontLeft.setPower(-gamepad1.left_stick_y+gamepad1.left_stick_x + gamepad1.right_stick_x);
        frontRight.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x - gamepad1.right_stick_x);
        backLeft.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x + gamepad1.right_stick_x);
        backRight.setPower(-gamepad1.left_stick_y+gamepad1.left_stick_x - gamepad1.right_stick_x);
    }
    }

