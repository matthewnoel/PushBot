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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;

@Autonomous

public class AutoBluPerp extends LinearOpMode {

  private static DcMotor left_drive;
  private static DcMotor right_drive;
  private static DcMotor arm_lift;
  private static ColorSensor color_prox;
  private static Servo right_thumb;
  private static Servo left_thumb;
  private static Servo ball_arm;
  private GyroSensor mr_gyro;
  private String glyphPosition;
  static BNO055IMU imu;

//START VUFORIA CODE
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
//END VUFORIA CODE

    @Override

    public void runOpMode() {

//START IMU STUFF
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        params.loggingEnabled      = true;
        params.loggingTag          = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
//END IMU STUFF

//START VUFORIA CODE
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AZitt3z/////AAAAGWgYGYTsHEwUkFKDfWt8S3kBi3Hjcf1sMDuVCJ5RH2a1dYhRjqbWtEB+FFlaLXM0812YvMXHTvCBmKtnJFsZZFNBk47tABdNJOtA+8T0jh9HdcGLZiyQ27ZAtsjln+EL4mo1pljJQ8lZvOeax6KxPJ+s5kRaaHfOAQLmEAKFu8o0QC1TaQ1P+K9O/+05/adVA5XKp1xrxn71fmzv1/VbZmGWUBJS5WHaWw5rz/zxC+T9mFXo98ErHggILR2YdGgkV0Cwe0ecG8pTbE3JU3FBo0LVOFzjFGUX8hBaw2WtSc8VdTKpsPQt6y6hT4IsgdieUUOWlchp8IoymcoVnS/wooeOL9SFD6OiWVmAKzFL37ND";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
//END VUFORIA CODE

        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
        color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
        right_thumb = hardwareMap.get(Servo.class, "right_thumb");
        left_thumb = hardwareMap.get(Servo.class, "left_thumb");
        ball_arm = hardwareMap.get(Servo.class, "ball_arm");
        mr_gyro = hardwareMap.get(GyroSensor.class, "mr_gyro");

        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lift.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        ball_arm.setPosition(0);
        pickUpGlyph();
        knockOffRed();
        moveToSafe();

        while (opModeIsActive()) {

//START VUFORIA CODE
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//I wrote this
            // saves glyph position as string in case visible one changes
            if(glyphPosition == null && vuMark != RelicRecoveryVuMark.UNKNOWN){
                glyphPosition = vuMark.toString();
            }
//End of what I wrote
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("glyphPosition",glyphPosition );

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
//END VUFORIA CODE

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

//START VUFORIA CODE
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
//END VUFORIA CODE

    // brings down arm, knocks of blue ball, brings arm up
    private void knockOffRed(){
        // drops arm down
        while(ball_arm.getPosition() < 0.62){
            ball_arm.setPosition(ball_arm.getPosition()+0.001);
        }

        // checks the color
        if(color_prox.red() > color_prox.blue()){
            //rotate left and knock off red
            while(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 20){
                    right_drive.setPower(0.125);
                    left_drive.setPower(-0.125);
            }
            // brings arm back up
            ball_arm.setPosition(0);
            left_drive.setPower(0);
            right_drive.setPower(0);
            while(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 20){
                right_drive.setPower(-0.125);
                left_drive.setPower(0.125);
            }
            right_drive.setPower(0);
            left_drive.setPower(0);
        } else {
            //rotate right and knock off red
            while(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 20){
                left_drive.setPower(0.125);
                right_drive.setPower(-0.125);
            }
            // brings arm back up
            ball_arm.setPosition(0);
            right_drive.setPower(0);
            left_drive.setPower(0);
            while(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 20){
                left_drive.setPower(-0.125);
                right_drive.setPower(0.125);
            }
            left_drive.setPower(0);
            right_drive.setPower(0);
        }
    }

    // moves to safe zone for blue team perpendicular layout
    private void moveToSafe(){
        //rotate right
            while(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 80){
                //left_drive.setPower(0.125);
                right_drive.setPower(0.125);

            }
            right_drive.setPower(0);
            left_drive.setPower(0);
        //move forward
        int encStartTwo = left_drive.getCurrentPosition();
        while(left_drive.getCurrentPosition() < encStartTwo + 5000){
            left_drive.setPower(0.5);
            right_drive.setPower(0.5);
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    private void pickUpGlyph(){
      while(mr_gyro.getHeading() < 2){
        arm_lift.setPower(-0.125);
      }
      arm_lift.setPower(0);
      left_thumb.setPosition(0.5);
      right_thumb.setPosition(0.5);
      while(mr_gyro.getHeading() < 30){
        arm_lift.setPower(-0.25);
      }
      arm_lift.setPower(0);
    }
}
