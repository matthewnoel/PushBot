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

public class PushBotDriver extends LinearOpMode {
    private DcMotor arm_lift;
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DigitalChannel touch_senser;
    private ColorSensor color_distance;
    private Servo right_thumb;
    private Servo left_thumb;
    private Servo ball_arm;
    private double thumbSpeed;
    private GyroSensor mr_gyro;
    private double speedDivisor;

    private Servo wrist_servo;
    private GyroSensor wrist_gyro;

    @Override

    public void runOpMode() {

        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        color_distance = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        mr_gyro = hardwareMap.get(GyroSensor.class, "mr_gyro");

        wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        wrist_gyro = hardwareMap.get(GyroSensor.class, "wrist_gyro");


        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ball_arm.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            //sets speed values using ternary operator to enable the precision modes
            speedDivisor = this.gamepad2.b?6:1.3;
            thumbSpeed = this.gamepad2.a?0.005:0.015;

            //Go to first height
            if(this.gamepad2.dpad_down){
                if(mr_gyro.getHeading() < 10 - 5 || mr_gyro.getHeading() > 10 + 5){
                    arm_lift.setPower(Math.signum(mr_gyro.getHeading() - 10) * 0.25);
                } else {
                    arm_lift.setPower(0);
                }
            }

            //Go to second height
            if(this.gamepad2.dpad_left){
                if(mr_gyro.getHeading() < 50 - 5 || mr_gyro.getHeading() > 50 + 5){
                    arm_lift.setPower(Math.signum(mr_gyro.getHeading() - 50) * 0.25);
                } else {
                    arm_lift.setPower(0);
                }
            }

            //Go to third height
            if(this.gamepad2.dpad_right){
                if(mr_gyro.getHeading() < 75 - 5 || mr_gyro.getHeading() > 75 + 5){
                    arm_lift.setPower(Math.signum(mr_gyro.getHeading() - 75) * 0.25);
                } else {
                    arm_lift.setPower(0);
                }
            }

            //Go to fourth height
            if(this.gamepad2.dpad_up){
                if(mr_gyro.getHeading() < 95 - 5 || mr_gyro.getHeading() > 95 + 5){
                    arm_lift.setPower(Math.signum(mr_gyro.getHeading() - 95) * 0.25);
                } else {
                    arm_lift.setPower(0);
                }
            }

            //opens and closes thumbs with buttons
            if(!this.gamepad1.right_bumper){
                if(this.gamepad1.right_trigger > 0.5){
                    right_thumb.setPosition(right_thumb.getPosition() + thumbSpeed);
                    left_thumb.setPosition(left_thumb.getPosition() - thumbSpeed);
                }
            } else if (this.gamepad1.right_bumper){
                right_thumb.setPosition(right_thumb.getPosition() - thumbSpeed);
                left_thumb.setPosition(left_thumb.getPosition() + thumbSpeed);
            }

            //sets arm power if the second driver is not using one of the automatic heights
            if(!this.gamepad2.dpad_down && !this.gamepad2.dpad_up && !this.gamepad2.dpad_left && !this.gamepad2.dpad_right){
                    if(mr_gyro < 10 && this.gamepad1.right_stick_y < 0){
                            arm_lift.setPower(0);
                    } else {
                        arm_lift.setPower(this.gamepad1.right_stick_y / speedDivisor);
                    }
            }

            //movement based on left stick
            left_drive.setPower(this.gamepad1.left_stick_y / speedDivisor - this.gamepad1.left_stick_x / speedDivisor);
            right_drive.setPower(-(this.gamepad1.left_stick_y / speedDivisor + this.gamepad1.left_stick_x / speedDivisor));

            // Uses wrist servo to keep hand level.
            if(wrist_gyro.getHeading < 34){
                    wrist_servo.setPosition(wrist_servo.getPosition() + 0.002);
            } else if(wrist_gyro.getHeading > 36){
                    wrist_servo.setPosition(wrist_servo.getPosition() - 0.002);
            }

            telemetry.addData("(B) Motor Precision Mode: ", this.gamepad2.b?"ON":"OFF");
            telemetry.addData("(A) Thumb Precision Mode: ", this.gamepad2.a?"ON":"OFF");
            telemetry.addData("(^) FOURTH Arm Height: ", this.gamepad2.dpad_up?"ON":"OFF");
            telemetry.addData("(>) THIRD Arm Height: ", this.gamepad2.dpad_right?"ON":"OFF");
            telemetry.addData("(<) SECOND Arm Height: ", this.gamepad2.dpad_left?"ON":"OFF");
            telemetry.addData("(⌄) FIRST Arm Height: ", this.gamepad2.dpad_down?"ON":"OFF");
            telemetry.update();
        }
    }
}
