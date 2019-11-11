/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevSPARKMini;

@TeleOp(name="Mecanum", group="Iterative Opmode")
public class FTCMecanumTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotorSimple AM = null;
    private Servo clamp = null;

    private ColorSensor CS = null;

    private Servo LSV = null;
    private Servo RSV = null;

    public void init() {
        telemetry.addData("Status", "Initialized");

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        AM = hardwareMap.get(DcMotorSimple.class, "AM");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        CS = hardwareMap.get(ColorSensor.class, "CS");
        AM = hardwareMap.get(DcMotorSimple.class, "AM");
        clamp = hardwareMap.get(Servo.class, "clamp");

        LSV = hardwareMap.get(Servo.class, "LSV");
        RSV = hardwareMap.get(Servo.class, "RSV");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void init_loop() {
    }

    public void start() {
        runtime.reset();
    }

    public void loop() {
        double Speed = -gamepad1.right_stick_y;
        double Turn = -gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        double MAX_SPEED = 1.0;

        if(gamepad1.left_bumper){
            LSV.setPosition(0.5);
            RSV.setPosition(0.5);
        }else if(gamepad1.right_bumper){
            LSV.setPosition(0.0);
            RSV.setPosition(0.0);
        }

        if(gamepad2.a){
            clamp.setPosition(0.0);
        }else if(gamepad2.x){
            clamp.setPosition(.65);
        }else if(gamepad2.y){
            clamp.setPosition(0.7);
        }


        if((FL.getCurrentPosition() >= 1075) && (gamepad2.right_stick_y < 0)){
            AM.setPower(0.0);
        }else if((FL.getCurrentPosition() <= 15) && (gamepad2.right_stick_y > 0)){
            AM.setPower(0.0);
        }else{
            if(Math.abs(gamepad2.right_stick_y) < 0.1){
                AM.setPower(-0.07);
            }else{
                AM.setPower(gamepad2.right_stick_y * 0.7);
            }
        }

        holonomic(Speed, Turn, Strafe, MAX_SPEED);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Color", CS.red());
        telemetry.addData("Arm Encoder", FL.getCurrentPosition());
    }

    public static double scaleInput(double number) {
        return Range.clip(number, -1.0, 1.0);
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {

//        FL = +Speed + Turn - Strafe      FR = +Speed - Turn + Strafe
//        BL  = +Speed + Turn + Strafe     BR  = +Speed - Turn - Strafe

        double Magnitude = Math.abs(Speed) + Math.abs(Turn) + Math.abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        FL.setPower(Range.scale((scaleInput(Speed) + scaleInput(Turn) - scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (BL != null) {
            BL.setPower(Range.scale((scaleInput(Speed) + scaleInput(Turn) + scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        FR.setPower(Range.scale((scaleInput(Speed) - scaleInput(Turn) + scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (BR != null) {
            BR.setPower(Range.scale((scaleInput(Speed) - scaleInput(Turn) - scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));


        }
    }
    public void stop() {

    }
}