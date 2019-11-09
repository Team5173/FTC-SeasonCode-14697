package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedSkyStone")
public class RedSkyStone extends LinearOpMode {

    private ColorSensor CS;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private Servo RSV = null;
    private Servo LSV = null;
    private DcMotorSimple AM;
    boolean isred = false;
    private Servo clamp;


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
        clamp = hardwareMap.get(Servo.class, "clamp");

        AM = hardwareMap.get(DcMotorSimple.class, "AM");


        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        clamp.setPosition(0.0);

        //Go Forward to the SkyStone
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);

        sleep(1550);

        //Stop motors and clamp the SkyStone
        stopMotor();

        clamp.setPosition(.7);

        sleep(1000);

        //Go backwards a little
        FL.setPower(-1);
        FR.setPower(-1);
        BL.setPower(-1);
        BR.setPower(-1);
        sleep(575);

        stopMotor();

        sleep(1000);

        //Turn right to the red line
        FL.setPower(-1);
        FR.setPower(1);
        BL.setPower(-1);
        BR.setPower(1);

        sleep(750);

        stopMotor();

        sleep(1000);

        //Go forward till the red line
        while(!isred) {
            isred = (CS.red()>100);

            FL.setPower(0.65);
            FR.setPower(0.65);
            BL.setPower(0.65);
            BR.setPower(0.65);

        }
        FL.setPower(0.65);
        FR.setPower(0.65);
        BL.setPower(0.65);
        BR.setPower(0.65);

        sleep(1000);
        isred = false;
        while(!isred) {
            isred = (CS.red()>100);

            FL.setPower(-0.65);
            FR.setPower(-0.65);
            BL.setPower(-0.65);
            BR.setPower(-0.65);

        }

        // stop motors
        stopMotor();

            telemetry.addData("Color", CS.red());
            telemetry.update();
    }

    public void stopMotor(){
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

}