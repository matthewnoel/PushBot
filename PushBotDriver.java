/*
Copyright 2017

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.loaderbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class PushBotDriver extends LinearOpMode {
    private DcMotor arm_lift;
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DigitalChannel touch_senser;
    private ColorSensor color_distance;
    private Servo right_thumb;
    private Servo left_thumb;
    private boolean bIsPressed = true;

    @Override
    public void runOpMode() {
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        //touch_senser = hardwareMap.get(DigitalChannel.class, "touch_senser");
        //color_distance = hardwareMap.get(ColorSensor.class, "color_distance");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(this.gamepad2.b){
                bIsPressed = false;
            } else {
                bIsPressed = true;
            }

            //Thumb code
            if(bIsPressed){
                //Dependant thumbs

                //Thumb code
                if(!this.gamepad1.right_bumper){
                    if(this.gamepad1.right_trigger > 0.5){
                        right_thumb.setPosition(right_thumb.getPosition() + 0.005);
                        left_thumb.setPosition(left_thumb.getPosition() - 0.005);
                    }
                } else if (this.gamepad1.right_bumper){
                    right_thumb.setPosition(right_thumb.getPosition() - 0.005);
                    left_thumb.setPosition(left_thumb.getPosition() + 0.005);
                }
            } else {
                //Independant thumbs

                //Left thumb code
                if(!this.gamepad1.left_bumper){
                    if(this.gamepad1.left_trigger > 0.5){
                        left_thumb.setPosition(left_thumb.getPosition() - 0.005);
                    }
                } else if (this.gamepad1.left_bumper){
                    left_thumb.setPosition(left_thumb.getPosition() + 0.005);
                }

                //Right thumb code
                if(!this.gamepad1.right_bumper){
                    if(this.gamepad1.right_trigger > 0.5){
                        right_thumb.setPosition(right_thumb.getPosition() + 0.005);
                    }
                } else if (this.gamepad1.right_bumper){
                    right_thumb.setPosition(right_thumb.getPosition() - 0.005);
                }
            }

            //Motor test code
            arm_lift.setPower(this.gamepad1.right_stick_y / 1.5);
            left_drive.setPower(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x);
            right_drive.setPower(-(this.gamepad1.left_stick_y + this.gamepad1.left_stick_x));
            /*
            telemetry.addData("Status", "Running");
            telemetry.addData("Boom Button Pressed", !touch_senser.getState());
            telemetry.addData("Red", color_distance.red());
            telemetry.addData("Green", color_distance.green());
            telemetry.addData("Blue", color_distance.blue());
            telemetry.addData("Alpha", color_distance.alpha());
            telemetry.update();
            */
            telemetry.addData("bool",bIsPressed);
            telemetry.update();
        }
    }
}
