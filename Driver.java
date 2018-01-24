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
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

public class Driver extends LinearOpMode {

        /**
         * Calibrates gyro.
         */
        public class CalibrateGyro implements StateMachine.State {
            @Override
            public void start() {
                wrist_gyro.calibrate();
                arm_gyro.calibrate();
                body_gyro.calibrate();
            }

            @Override
            public State update() {
                if(wrist_gyro.isCalibrating() || arm_gyro.isCalibrating() || body_gyro.isCalibrating()){
                        return this;
                } else {
                        return drive;
                }
            }
        }

        /**
         * Controlls all the stuff.
         */
        public class Drive implements StateMachine.State {
            @Override
            public void start() {
                ball_arm.setPosition(0);
                left_thumb.setPosition(0.5);
                right_thumb.setPosition(0.5);
                wrist_servo.setPosition(1);
            }

            @Override
            public State update() {
                    //sets speed values using ternary operator to enable the precision modes
                    speedDivisor = gamepad2.b?6:1.3;
                    thumbSpeed = gamepad2.a?0.005:0.03;
/*
                    //Go to first height
                    if(gamepad2.dpad_down){
                        if(arm_gyro.getHeading() < 10 - 5 || arm_gyro.getHeading() > 10 + 5){
                            arm_lift.setPower(Math.signum(arm_gyro.getHeading() - 10) * 0.25);
                        } else {
                            arm_lift.setPower(0);
                        }
                    }

                    //Go to second height
                    if(gamepad2.dpad_left){
                        if(arm_gyro.getHeading() < 50 - 5 || arm_gyro.getHeading() > 50 + 5){
                            arm_lift.setPower(Math.signum(arm_gyro.getHeading() - 50) * 0.25);
                        } else {
                            arm_lift.setPower(0);
                        }
                    }

                    //Go to third height
                    if(gamepad2.dpad_right){
                        if(arm_gyro.getHeading() < 75 - 5 || arm_gyro.getHeading() > 75 + 5){
                            arm_lift.setPower(Math.signum(arm_gyro.getHeading() - 75) * 0.25);
                        } else {
                            arm_lift.setPower(0);
                        }
                    }

                    //Go to fourth height
                    if(gamepad2.dpad_up){
                        if(arm_gyro.getHeading() < 95 - 5 || arm_gyro.getHeading() > 95 + 5){
                            arm_lift.setPower(Math.signum(arm_gyro.getHeading() - 95) * 0.25);
                        } else {
                            arm_lift.setPower(0);
                        }
                    }
*/
                    //opens and closes thumbs with buttons
                    if(gamepad1.right_trigger < 0.5){
                        if(gamepad1.right_bumper){
                            right_thumb.setPosition(right_thumb.getPosition() + thumbSpeed);
                            left_thumb.setPosition(left_thumb.getPosition() - thumbSpeed);
                        }
                    } else if (gamepad1.right_trigger > 0.5){
                        right_thumb.setPosition(right_thumb.getPosition() - thumbSpeed);
                        left_thumb.setPosition(left_thumb.getPosition() + thumbSpeed);
                    }
/*
                    //sets arm power if the second driver is not using one of the automatic heights
                    if(!gamepad2.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_right){
                            if(gamepad1.right_stick_y < 0 && arm_gyro.getHeading() < 3){
                                    arm_lift.setPower(0);
                            } else {
                                arm_lift.setPower(gamepad1.right_stick_y / speedDivisor);
                            }
                    }
*/

                    arm_lift.setPower(gamepad1.right_stick_y * 0.5 / speedDivisor);

                    //movement based on left stick
                    back_left.setPower(gamepad1.left_stick_y / speedDivisor - gamepad1.left_stick_x / speedDivisor);
                    front_left.setPower(gamepad1.left_stick_y / speedDivisor - gamepad1.left_stick_x / speedDivisor);
                    back_right.setPower(-(gamepad1.left_stick_y / speedDivisor + gamepad1.left_stick_x / speedDivisor));
                    front_right.setPower(-(gamepad1.left_stick_y / speedDivisor + gamepad1.left_stick_x / speedDivisor));

                    //levels out the wrist.
                    if(wrist_gyro.getHeading() < 68){
                        wrist_servo.setPosition(wrist_servo.getPosition() - 0.01);
                    }else if(wrist_gyro.getHeading() > 71){
                        wrist_servo.setPosition(wrist_servo.getPosition() + 0.01);
                    }

                    telemetry.addData("(B) Motor Precision Mode: ", gamepad2.b?"ON":"OFF");
                    telemetry.addData("(A) Thumb Precision Mode: ", gamepad2.a?"ON":"OFF");
                    telemetry.addData("(^) FOURTH Arm Height: ", gamepad2.dpad_up?"ON":"OFF");
                    telemetry.addData("(>) THIRD Arm Height: ", gamepad2.dpad_right?"ON":"OFF");
                    telemetry.addData("(<) SECOND Arm Height: ", gamepad2.dpad_left?"ON":"OFF");
                    telemetry.addData("(⌄) FIRST Arm Height: ", gamepad2.dpad_down?"ON":"OFF");
                    telemetry.update();

                return this;
            }
        }

    @Override

    public void runOpMode() {

        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        arm_gyro = hardwareMap.get(GyroSensor.class, "arm_gyro");
        wrist_gyro = hardwareMap.get(GyroSensor.class, "wrist_gyro");
        body_gyro = hardwareMap.get(GyroSensor.class, "body_gyro");

        calibrateGyro = new CalibrateGyro();
        drive = new Drive();
        machine = new StateMachine(calibrateGyro);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }

    private DcMotor arm_lift;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private ColorSensor color_prox;
    private Servo wrist_servo;
    private Servo right_thumb;
    private Servo left_thumb;
    private Servo ball_arm;
    private GyroSensor arm_gyro;
    private GyroSensor wrist_gyro;
    private GyroSensor body_gyro;
    private double thumbSpeed;
    private double speedDivisor;

    private CalibrateGyro calibrateGyro;
    private Drive drive;
    private StateMachine machine;
}
