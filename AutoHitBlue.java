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
package org.firstinspires.ftc.pushbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

public class AutoHitBlue extends LinearOpMode {
    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor arm_lift;
    private ColorSensor color_prox;
    private Servo right_thumb;
    private Servo left_thumb;
    private Servo ball_arm;
    private boolean isInPlace = false;


    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        ball_arm = hardwareMap.get(Servo.class, "ball_arm");

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //Color sensor code
        ball_arm.setPosition(0);
        
        
        
        if(color_prox.blue() > 10){
            //left_drive.setPower(-1);
            //right_drive.setPower(1);
            /*
            for(int i = 0; i < 50; i++){
                left_drive.setPower(-1);
                right_drive.setPower(1);
            }
            left_drive.setPower(0);
            right_drive.setPower(0);
            */

            
            
            
        }else{
            //left_drive.setPower(-1);
            //right_drive.setPower(1);
            /*
            for(int i = 0; i < 50; i++){
                left_drive.setPower(1);
                right_drive.setPower(11);
            }
            left_drive.setPower(0);
            right_drive.setPower(0);
            */

        }
        
        //Drive to the point where we need to be

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(ball_arm.getPosition() < 0.74){
                ball_arm.setPosition(ball_arm.getPosition()+0.01);
            } else {
                isInPlace = true;
            }
            
            if(isInPlace){
                    
                if(color_prox.blue() > 10){
                //left_drive.setPower(-1);
                //right_drive.setPower(1);
                /*
                for(int i = 0; i < 50; i++){
                    left_drive.setPower(-1);
                    right_drive.setPower(1);
                }
                left_drive.setPower(0);
                right_drive.setPower(0);
                */
            if(right_drive.getCurrentPosition() < 500){
                right_drive.setPower(0.5);
                
            }
            if(left_drive.getCurrentPosition() > -500){
                left_drive.setPower(-0.5);
                
            }
                }else{
                //left_drive.setPower(-1);
                //right_drive.setPower(1);
                /*
                for(int i = 0; i < 50; i++){
                    left_drive.setPower(1);
                    right_drive.setPower(11);
                }
                left_drive.setPower(0);
                right_drive.setPower(0);
                */
            if(left_drive.getCurrentPosition() < 500){
                left_drive.setPower(0.5);
                
            }
            if(right_drive.getCurrentPosition() > -500){
                right_drive.setPower(0.5);
                
            }
                }
            }
            
            telemetry.addData("Status", "Running");
            telemetry.addData("Red", color_prox.red());
            telemetry.addData("Green", color_prox.green());
            telemetry.addData("Blue", color_prox.blue());
            telemetry.addData("Servo", ball_arm.getPosition());
            telemetry.addData("Left Motor",left_drive.getCurrentPosition());
            telemetry.addData("Right Motor",right_drive.getCurrentPosition());
            telemetry.update();

        }
    }
}
