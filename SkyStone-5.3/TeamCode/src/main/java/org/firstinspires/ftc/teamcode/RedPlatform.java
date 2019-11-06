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
    private DcMotorSimple AM;
    boolean isred = false;


    public void runOpMode() throws InterruptedException {
        final int ARM_RAISE_TIME = 350;
        final int ARM_LOWER_TIME = 200;

        CS= hardwareMap.get(ColorSensor.class , "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");

        AM = hardwareMap.get(DcMotorSimple.class, "AM");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        AM.setPower(-0.5);                                                      // Start arm raise

        //Go Forward to the Platform
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);

        sleep(ARM_RAISE_TIME);                                                  // Drive and raise simultaneously
        AM.setPower(-0.07);                                                     // Arm hold current

        sleep(1550 - ARM_RAISE_TIME);

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
        FL.setPower(-1);
        FR.setPower(-1);
        BL.setPower(-1);
        BR.setPower(-1);

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

        //goes left for one second
        FL.setPower(-0.65);
        FR.setPower(0.65);
        BL.setPower(0.65);
        BR.setPower(-0.65);

        sleep(1000-ARM_LOWER_TIME);
        AM.setPower(0.25);                                                      // Start lowering arm
        sleep(ARM_LOWER_TIME);
        AM.setPower(0);

        //Go left till the red line
        while(!isred) {
            isred = (CS.red()>100);

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