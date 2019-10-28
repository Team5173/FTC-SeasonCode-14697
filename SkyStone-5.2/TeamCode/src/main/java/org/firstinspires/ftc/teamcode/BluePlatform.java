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
    boolean isblue = false;


    public void runOpMode() throws InterruptedException {

        CS= hardwareMap.get(ColorSensor.class , "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //Go Forward to the Platform
        FL.setPower(-1);
        FR.setPower(-1);
        BL.setPower(-1);
        BR.setPower(-1);

        sleep(1750);

        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        sleep(1000);

        //Set the servos to grab the platform
        LSV.setPosition(0.6);
        RSV.setPosition(0.6);

        sleep(1000);

        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        sleep(1000);

        //Go Backwards while pulling the platform
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);

        sleep(2750);

        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        //Set servos so we can slide away from the platform
        LSV.setPosition(0.0);
        RSV.setPosition(0.0);

        sleep(2000);

        //goes right for one second
        FL.setPower(-0.65);
        FR.setPower(0.65);
        BL.setPower(0.65);
        BR.setPower(-0.65);

        sleep(1000);

        //Go right until the red line
        while(!isblue) {
            isblue = (CS.blue()>100);

            FL.setPower(-0.65);
            FR.setPower(0.65);
            BL.setPower(0.65);
            BR.setPower(-0.65);

        }
        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        telemetry.addData("Color", CS.red());
        telemetry.update();
    }
}