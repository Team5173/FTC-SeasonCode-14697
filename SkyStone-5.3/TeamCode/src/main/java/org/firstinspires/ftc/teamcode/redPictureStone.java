package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "RedPictureStone")
public class redPictureStone extends LinearOpMode {

    private ColorSensor CS;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private Servo RSV = null;
    private Servo LSV = null;
    private DcMotorSimple AM = null;
    private Servo clamp = null;
    private DistanceSensor DS = null;
    boolean isblue = false;
    private ColorSensor CSS;


    public void runOpMode() throws InterruptedException {

        CS = hardwareMap.get(ColorSensor.class, "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        AM = hardwareMap.get(DcMotorSimple.class, "AM");
        clamp = hardwareMap.get(Servo.class, "clamp");
        DS = hardwareMap.get (DistanceSensor.class, "DS");
        CSS = hardwareMap.get (ColorSensor.class,"CSS");
        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while(DS.getDistance(DistanceUnit.CM) >5){
            goForward(0.35);
        }
        stopMotor();
        sleep(250);

        while(CSS.red() >125 && CSS.green() >125 && CSS.blue() <25) {

        strafeLeft(0.35);

        }
        stopMotor();
        sleep(450);

        strafeLeft(0.35);
        sleep(125);
        stopMotor();
        sleep(125);
        goForward(0.35);
        sleep(74);
        stopMotor();
        clamp.setPosition(.7);
        sleep(250);
        goBackward(0.35);
        sleep(199);
        stopMotor();
        sleep(74);
        turnRight();
        sleep(375);
        //Go forward until the blue line
        while (!isblue) {
            isblue = (CS.blue() > 100);

            goForward(0.4);

        }

        goForward(0.5);

        sleep(500);
        isblue = false;
        while (!isblue) {
            isblue = (CS.blue() > 100);

            goBackward(0.4);

        }
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
        FL.setPower(sp);
        FR.setPower(-sp);
        BL.setPower(-sp);
        BR.setPower(sp);
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
