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
            }

            @Override
            public State update() {
                if(wrist_gyro.isCalibrating()){
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
            }

            @Override
            public State update() {
                //movement based on left stick
                left_drive.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                right_drive.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x));
                left_front_drive.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                right_front_drive.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x));

                /*
                telemetry.addData("(B) Motor Precision Mode: ", gamepad2.b?"ON":"OFF");
                telemetry.addData("(A) Thumb Precision Mode: ", gamepad2.a?"ON":"OFF");
                telemetry.addData("(^) FOURTH Arm Height: ", gamepad2.dpad_up?"ON":"OFF");
                telemetry.addData("(>) THIRD Arm Height: ", gamepad2.dpad_right?"ON":"OFF");
                telemetry.addData("(<) SECOND Arm Height: ", gamepad2.dpad_left?"ON":"OFF");
                telemetry.addData("(⌄) FIRST Arm Height: ", gamepad2.dpad_down?"ON":"OFF");
                telemetry.update();
                */

                return this;
            }
        }

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
        ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        //mr_gyro = hardwareMap.get(GyroSensor.class, "mr_gyro");
        //wrist_servo = hardwareMap.get(Servo.class, "wrist_servo");
        wrist_gyro = hardwareMap.get(GyroSensor.class, "wrist_gyro");

        calibrateGyro = new CalibrateGyro();
        drive = new Drive();
        machine = new StateMachine(calibrateGyro);

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

        waitForStart();

        while (opModeIsActive()) {
                machine.update();
        }
    }

    private DcMotor arm_lift;
    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor right_front_drive;
    private DcMotor left_front_drive;
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

    private CalibrateGyro calibrateGyro;
    private Drive drive;
    private StateMachine machine;
}
