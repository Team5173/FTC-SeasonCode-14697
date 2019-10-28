package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "BlueSide Basic")
public class BlueSideBasic extends LinearOpMode {

    private ColorSensor CS;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    boolean isBlue = false;


    public void runOpMode() throws InterruptedException {

        CS= hardwareMap.get(ColorSensor.class , "CS");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //Go right to the blue line
        while(!isBlue) {
            isBlue = (CS.blue()>100);

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

        telemetry.addData("Color", CS.blue());
        telemetry.update();
    }
}