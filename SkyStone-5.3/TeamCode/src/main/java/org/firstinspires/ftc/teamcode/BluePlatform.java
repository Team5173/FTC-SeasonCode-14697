package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "BlueSide Platform")
public class BluePlatform extends LinearOpMode {

    private ColorSensor CS;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private Servo RSV = null;
    private Servo LSV = null;
    private DcMotorSimple AM = null;
    boolean isblue = false;


    public void runOpMode() throws InterruptedException {

        CS = hardwareMap.get(ColorSensor.class, "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        AM = hardwareMap.get(DcMotorSimple.class, "AM");

        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        final int ARM_RAISE_TIME = 350;
        final int ARM_LOWER_TIME = 200;

        waitForStart();

        AM.setPower(-0.5);

        //Go Forward to the Platform
        goForward(0.65);

        sleep(ARM_RAISE_TIME);

        AM.setPower(-0.07);

        sleep(675 - ARM_RAISE_TIME);

        // stop motors
        stopMotor();
        sleep(500);

        //Set the servos to grab the platform
        LSV.setPosition(0.6);
        RSV.setPosition(0.6);

        sleep(500);

        //Go left for an eigth of a second
        strafeLeft(0.65);

        sleep(400);

        //Go Backwards while pulling the platform
        goBackward(0.65);

        sleep(1375);

        // stop motors
        stopMotor();

        //Set servos so we can slide away from the platform
        LSV.setPosition(0.0);
        RSV.setPosition(0.0);

        sleep(1000);

        //goes right for one and a half seconds
        strafeRight(0.65);

        sleep(750 - ARM_LOWER_TIME);

        AM.setPower(0.25);

        sleep(ARM_LOWER_TIME);

        AM.setPower(0);

        //Go right until the red line
        while (!isblue) {
            isblue = (CS.blue() > 100);

           strafeRight(0.4);

        }
        // stop motors
        stopMotor();

        telemetry.addData("Color", CS.red());
        telemetry.update();
    }

    public void stopMotor() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    public void goBackward(double sp) {
        FL.setPower(sp);
        FR.setPower(sp);
        BL.setPower(sp);
        BR.setPower(sp);
    }

    public void strafeLeft(double sp) {
        FL.setPower(sp);
        FR.setPower(-sp);
        BL.setPower(-sp);
        BR.setPower(sp);

    }

    public void goForward(double sp) {
        FL.setPower(-sp);
        FR.setPower(-sp);
        BL.setPower(-sp);
        BR.setPower(-sp);
    }
    public void strafeRight(double sp) {
        FL.setPower(-sp);
        FR.setPower(sp);
        BL.setPower(sp);
        BR.setPower(-sp);
    }


    public void turnLeft() {
        FL.setPower(1);
        FR.setPower(-1);
        BL.setPower(1);
        BR.setPower(-1);
    }


    public void turnRight() {
        FL.setPower(-1);
        FR.setPower(1);
        BL.setPower(-1);
        BR.setPower(1);
    }
}