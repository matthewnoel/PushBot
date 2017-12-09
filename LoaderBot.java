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
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
@Disabled

public class LoaderBot extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor boom_lift;
    private DcMotor bucket_tip;
    private DigitalChannel boom_limit;
    private Servo bucket_thumb;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        boom_lift = hardwareMap.get(DcMotor.class, "boom_lift");
        bucket_tip = hardwareMap.get(DcMotor.class, "bucket_tip");
        boom_limit = hardwareMap.get(DigitalChannel.class, "boom_limit");
        bucket_thumb = hardwareMap.get(Servo.class, "bucket_thumb");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // is button pressed?
            if (boom_limit.getState() == false) {
                // button is pressed.
                telemetry.addData("Button", "PRESSED");
                boom_lift.setPower(0);
                if(this.gamepad1.dpad_up){
                    boom_lift.setPower(-.5);

                }

            } else {
                // button is not pressed.
                telemetry.addData("Button", "NOT PRESSED");
                if(this.gamepad1.dpad_up){
                  boom_lift.setPower(-.5);
                } else if(this.gamepad1.dpad_down){
                    boom_lift.setPower(.5);
                } else {
                    boom_lift.setPower(0);
                }


            }
            left_drive.setPower(this.gamepad1.left_stick_y);
            right_drive.setPower(-this.gamepad1.right_stick_y);

            telemetry.addData("Left Stick", this.gamepad1.left_stick_y);
            telemetry.addData("Right Stick", this.gamepad1.right_stick_y);
            telemetry.addData("Left Power", left_drive.getPower());
            telemetry.addData("Right Power", right_drive.getPower());
            telemetry.addData("Boom Encoder", boom_lift.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
