package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedSide Platform")
public class RedPlatform extends LinearOpMode {

    private ColorSensor CS;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private Servo RSV = null;
    private Servo LSV = null;
    private Servo clamp;
    private DcMotorSimple AM;
    boolean isred = false;


    public void runOpMode() throws InterruptedException {
        final int ARM_RAISE_TIME = 350;
        final int ARM_LOWER_TIME = 200;

        CS = hardwareMap.get(ColorSensor.class, "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");
        clamp = hardwareMap.get(Servo.class, "clamp");

        AM = hardwareMap.get(DcMotorSimple.class, "AM");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        AM.setPower(-0.5);                                                      // Start arm raise

        //Go Forward to the Platform

        goForward();
        sleep(ARM_RAISE_TIME);                                                  // Drive and raise simultaneously
        AM.setPower(-0.07);                                                     // Arm hold current
        clamp.setPosition(0.0);
        sleep(650 - ARM_RAISE_TIME);

        // stop motors

        stopMotor();
        sleep(500);

        //Set the servos to grab the platform
        LSV.setPosition(0.6);
        RSV.setPosition(0.6);

        sleep(500);

        //Go right for an eigth of a second

        strafeRight(0.65);
        sleep(250);

        // stop motors
        stopMotor();
        sleep(500);

        //Go Backwards while pulling the platform

        goBackward();
        sleep(1375);

        // stop motors
        stopMotor();

        //Set servos so we can slide away from the platform
        LSV.setPosition(0.0);
        RSV.setPosition(0.0);

        sleep(1000);

        //goes left for one and a half seconds

        strafeLeft(0.75);
        sleep(750 - ARM_LOWER_TIME);
        AM.setPower(0.25);                                                      // Start lowering arm
        sleep(ARM_LOWER_TIME);
        AM.setPower(0);

        //Go left until the red line
        while (!isred) {
            isred = (CS.red() > 100);


            strafeLeft(0.4);
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

    public void strafeRight(double sp) {
        FL.setPower(-sp);
        FR.setPower(sp);
        BL.setPower(sp);
        BR.setPower(-sp);

    }

    public void goForward() {
        FL.setPower(-0.58);
        FR.setPower(-0.58);
        BL.setPower(-0.58);
        BR.setPower(-0.58);
    }

    public void strafeLeft(double sp) {
        FL.setPower(sp);
        FR.setPower(-sp);
        BL.setPower(-sp);
        BR.setPower(sp);
    }
    public void goBackward(){
        FL.setPower(0.65);
        FR.setPower(0.65);
        BL.setPower(0.65);
        BR.setPower(0.65);
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
