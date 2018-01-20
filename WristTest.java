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

public class WristTest extends LinearOpMode {

    private Servo wrist_servo;
    private GyroSensor wrist_gyro;

    @Override

    public void runOpMode() {

        wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        wrist_gyro = hardwareMap.get(GyroSensor.class, "wrist_gyro");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
//wrist_gyro.calibrate();
        while (opModeIsActive() && !wrist_gyro.isCalibrating()) {

            //this is just a test to see if there is a delay
            //it should just go down if started in the upright position
            //wrist_servo.setPosition(wrist_servo.getPosition() - 0.001);
            wrist_servo.setPosition(0.5);

            //this is the actual level code v1
            /*
            if(wrist_gyro.getHeading() < 71 || wrist_gyro.getHeading() > 350){
                wrist_servo.setPosition(wrist_servo.getPosition() - 0.001);
            } else if(wrist_gyro.getHeading() > 69){
                wrist_servo.setPosition(wrist_servo.getPosition() + 0.001);
            }
            */

            telemetry.addData("Gyro heading", wrist_gyro.getHeading());
            telemetry.addData("Servo position", wrist_servo.getPosition());
            telemetry.update();
        }
    }
}
