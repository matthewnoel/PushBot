/*
Copyright 2017

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS
IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.loaderbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class TighterTurns extends LinearOpMode {
    //private DcMotor arm_lift;
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor right_front_drive;
    private DcMotor left_front_drive;
    private DigitalChannel touch_senser;
    private ColorSensor color_distance;
    private Servo right_thumb;
    private Servo left_thumb;
    private Servo ball_arm;
    //added
    private Servo wrist_servo;
    private double thumbSpeed;
    private GyroSensor wrist_gyro;
    private double speedDivisor;

    @Override

    public void runOpMode() {

        //arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        //color_distance = hardwareMap.get(ColorSensor.class, "color_prox");
        //right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        //left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        //added
        //wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        //ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        //wrist_gyro = hardwareMap.get(GyroSensor.class, "wrist_gyro");

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm_lift.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //movement based on left stick
            left_drive.setPower(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x);
            right_drive.setPower(-(this.gamepad1.left_stick_y + this.gamepad1.left_stick_x));
            left_front_drive.setPower(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x);
            right_front_drive.setPower(-(this.gamepad1.left_stick_y + this.gamepad1.left_stick_x));
        }
    }
}