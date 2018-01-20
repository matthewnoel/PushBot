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
//package org.firstinspires.ftc.loaderbot;
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.StateMachine.State;
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

public class StMchWristTest extends LinearOpMode {

        /**
         * Calibrates gyro.
         */
        public class CalibrateGyro implements StateMachine.State {
            @Override
            public void start() {
                wrist_gyro.calibrate();
            }

            @Override
            public State update() {
                if(wrist_gyro.isCalibrating()){
                        return this;
                } else {
                        wrist_servo.setPosition(1);

                        return bringWristDown;
                }
            }
        }

        /**
         * Brings servo down.
         */
        public class BringWristDown implements StateMachine.State {
            @Override
            public void start() {

            }

            @Override
            public State update() {
                if(wrist_servo.getPosition() > 0.6){
                    wrist_servo.setPosition(wrist_servo.getPosition() - 0.0005);
                    return this;
                } else {
                    return betterWristTest;
                }
            }
        }

        /**
         * Tests wrist.
         */
        public class TestWrist implements StateMachine.State {
            @Override
            public void start() {
                wrist_servo.setPosition(1);
            }

            @Override
            public State update() {
                if(wrist_gyro.getHeading() < 71 || wrist_gyro.getHeading() > 350){
                    wrist_servo.setPosition(wrist_servo.getPosition() - 0.001);
                } else if(wrist_gyro.getHeading() > 69){
                    wrist_servo.setPosition(wrist_servo.getPosition() + 0.001);
                }
                return this;
            }
        }

        /**
         * Test the wrist in the better way.
         */
        public class BetterWristTest implements StateMachine.State {

            double goal;
            double deltaTheta;

            @Override
            public void start() {
                goal = wrist_gyro.getHeading();
            }

            @Override
            public State update() {
                deltaTheta = wrist_gyro.getHeading() - goal;
                wrist_servo.setPosition(deltaTheta / 190.5 + wrist_servo.getPosition());

                return this;
            }
        }

    @Override

    public void runOpMode() {

        wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        wrist_gyro = hardwareMap.get(GyroSensor.class, "wrist_gyro");
        calibrateGyro = new CalibrateGyro();
        bringWristDown = new BringWristDown();
        testWrist = new TestWrist();
        betterWristTest = new BetterWristTest();
        machine = new StateMachine(calibrateGyro);

        waitForStart();

        while (opModeIsActive()) {
            machine.update();
        }
    }

    private Servo wrist_servo;
    private GyroSensor wrist_gyro;

    private StateMachine machine;
    private CalibrateGyro calibrateGyro;
    private BringWristDown bringWristDown;
    private TestWrist testWrist;
    private BetterWristTest betterWristTest;
}
