package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Sandbox {

    public void rotateRight(DcMotor left, DcMotor right, BNO055IMU imu, double degrees){
    //Left is Positive Right is Negative
    //Counterclock is Positve Clock is Negative
    left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -degrees){
            left.setPower(0.125);
            right.setPower(-0.125);
        }
        right.setPower(0);
        left.setPower(0);
   }

    public void rotateLeft(DcMotor left, DcMotor right, BNO055IMU imu, double degrees){
    //Left is Positive Right is Negative
    left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < degrees){
            left.setPower(-0.125);
            right.setPower(0.125);
        }
        right.setPower(0);
        left.setPower(0);
   }

   public void moveForward(DcMotor left, DcMotor right, double distance, double speed){
       left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int encStart = left.getCurrentPosition();
        while(left.getCurrentPosition() < encStart + distance){
            left.setPower(speed);
            right.setPower(speed);
        }
        left.setPower(0);
        right.setPower(0);
   }

    public void armDown(DcMotor arm, double distance, double speed){
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int encStart = arm.getCurrentPosition();
        while(arm.getCurrentPosition() < encStart + distance){
            arm.setPower(speed);
        }
        arm.setPower(0);
   }
}
