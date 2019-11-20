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
    private Servo clamp = null;
    boolean isblue = false;


    public void runOpMode() throws InterruptedException {

        CS = hardwareMap.get(ColorSensor.class, "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        AM = hardwareMap.get(DcMotorSimple.class, "AM");
        clamp = hardwareMap.get(Servo.class, "clamp");

        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");

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

        //Go backwards a little
       goBackward(0.65);
        sleep(287);

        stopMotor();

        sleep(500);

        //Turn left to the blue line
       turnLeft();

        sleep(375);

        stopMotor();

        sleep(500);

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
