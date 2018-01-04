package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

public class StMchAutoBluPerp extends LinearOpMode{

    /**
     * Rotates until angle is met.
     */
    public class RotateUntilAngle implements StateMachine.State {
        @Override
        public void start() {
        }

        @Override
        public State update() {
            return this;
        }
    }

    /**
     * Moves forward until a distance threshold is met.
     */
    public class ForwardUntilDistance implements StateMachine.State {
        @Override
        public void start() {
                encoderStart = left_drive.getCurrentPosition();
        }

        @Override
        public State update() {

                if (left_drive.getCurrentPosition() < encoderStart + 3000) {
                    // Haven't yet reached distance, drive forward.
                    left_drive.setPower(0.125);
                    right_drive.setPower(0.125);
                    return this;
                } else {
                    // Reached distance, switch to rotate state.
                    left_drive.setPower(0);
                    right_drive.setPower(0);
                    return null;
                }

        }
        private int encoderStart;
    }

    /**
     * Initializes the state machine.
     */
     @Override
     public void runOpMode(){
         //super.init();  // Initialize mecanum drive.

         // Create the states.


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

        forwardUntilDistance = new ForwardUntilDistance();
         rotateUntilAngle = new RotateUntilAngle();
         // Start the state machine with forward state.
         machine = new StateMachine(forwardUntilDistance);
         waitForStart();
         while(opModeIsActive()){
             machine.update();
         }
}

    /**
     * Runs the state machine.
     */


    // Hardware stuff.
    private static DcMotor left_drive;
    private static DcMotor right_drive;
    private static DcMotor arm_lift;
    private static ColorSensor color_prox;
    private static Servo right_thumb;
    private static Servo left_thumb;
    private static Servo ball_arm;
    private GyroSensor mr_gyro;
    //private String glyphPosition;
    static BNO055IMU imu;

    // The state machine manager.
    private StateMachine machine;
    // Rotate until angle.
    private RotateUntilAngle rotateUntilAngle;
    // Move forward until distance.
    private ForwardUntilDistance forwardUntilDistance;

}