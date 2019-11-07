package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "BlueSkyStone")
public class BlueSkyStone extends LinearOpMode {

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

        CS= hardwareMap.get(ColorSensor.class , "CS");
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



        //Go Forward to the SkyStone
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);

        sleep(1550);



        sleep(1550-ARM_RAISE_TIME);

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

        //goes right for one second
        FL.setPower(0.65);
        FR.setPower(-0.65);
        BL.setPower(-0.65);
        BR.setPower(0.65);

        sleep(1000-ARM_LOWER_TIME);

        AM.setPower(0.25);

        sleep(ARM_LOWER_TIME);

        AM.setPower(0);

        //Go right until the red line
        while(!isblue) {
            isblue = (CS.blue()>100);

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