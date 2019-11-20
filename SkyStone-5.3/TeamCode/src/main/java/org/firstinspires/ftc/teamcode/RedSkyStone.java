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
        goForward(0.65);

        sleep(775);

        //Stop motors and clamp the SkyStone
        stopMotor();

        clamp.setPosition(.7);

        sleep(500);

        //Go backwards a 0.5 seconds
        goBackward(0.4);
        sleep(287);

        stopMotor();

        sleep(250);

        //Turn right to the red line
        turnLeft();

        sleep(375);

        stopMotor();

        sleep(500);

        //Go forward until the red line
        while(!isred) {
            isred = (CS.red()>100);

           goForward(0.4);

        }
        goForward(0.5);

        sleep(500);
        isred = false;
        while(!isred) {
            isred = (CS.red()>100);

            goBackward(0.4);

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