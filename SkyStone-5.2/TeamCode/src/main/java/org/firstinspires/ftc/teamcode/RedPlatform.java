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
    boolean isred = false;


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

        FL.setPower(-1);
        FR.setPower(-1);
        BL.setPower(-1);
        BR.setPower(-1);

        sleep(2300);

        LSV.setPosition(0.5);
        RSV.setPosition(0.5);

        wait(1000);

        FL.setPower(-1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(-1);

        sleep(1500);

        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);

        sleep(3000);

        while(!isred) {
            isred = (CS.red()>100);

            FL.setPower(0.65);
            FR.setPower(-0.65);
            BL.setPower(-0.65);
            BR.setPower(0.65);

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