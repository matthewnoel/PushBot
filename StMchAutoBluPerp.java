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
   * Picks up glyph.
   */
  public class PickUpGlyph implements StateMachine.State {
      @Override
      public void start() {
              left_thumb.setPosition(0.5);
              right_thumb.setPosition(0.5);
      }

      @Override
      public State update() {
          if(mr_gyro.getHeading() < 40){
                  arm_lift.setPower(-0.25);
                  return this;
          } else {
                  arm_lift.setPower(0);
                  return lowerColorSensor;
          }
      }
  }

  /**
   * Lowers color sensor arm.
   */
  public class LowerColorSensor implements StateMachine.State {
      @Override
      public void start() {
              ball_arm.setPosition(0);
      }

      @Override
      public State update() {
          if(ball_arm.getPosition() < 0.62){
                  ball_arm.setPosition(ball_arm.getPosition()+0.001);
                  return this;
          } else {
                  return knockRedBallOff;
          }
      }
  }

  /**
   * Knocks red ball off.
   */
  public class KnockRedBallOff implements StateMachine.State {
      @Override
      public void start() {
              if(color_prox.red() > color_prox.blue()){
                      isLeft = true;
              } else {
                      isLeft = false;
              }
      }

      @Override
      public State update() {
              if(isLeft){
                      // Rotate left and knock off red ball.
                      if(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 20){
                              left_drive.setPower(-0.125);
                              right_drive.setPower(0.125);
                              return this;
                      } else {
                              left_drive.setPower(0);
                              right_drive.setPower(0);
                              return rotateUntilAngle;
                      }
              } else {
                      // Rotate right and knock off red ball.
                      if(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 20){
                              left_drive.setPower(0.125);
                              right_drive.setPower(-0.125);
                              return this;
                      } else {
                              left_drive.setPower(0);
                              right_drive.setPower(0);
                              return rotateUntilAngle;
                      }
              }
      }
      private boolean isLeft;
  }

    /**
     * Rotates until angle is met.
     */
    public class RotateUntilAngle implements StateMachine.State {
        @Override
        public void start() {
        }

        @Override
        public State update() {
            if(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 70){
                    left_drive.setPower(-0.125);
                    right_drive.setPower(0.25);
                    return this;
            } else {
                    left_drive.setPower(0);
                    right_drive.setPower(0);
                    return driveGlyphToSlot;
            }
        }
    }

    /**
     * Drives glyph to slot
     */
    public class DriveGlyphToSlot implements StateMachine.State {
        @Override
        public void start() {
          encoderStart = left_drive.getCurrentPosition();
        }

        @Override
        public State update() {
          if (left_drive.getCurrentPosition() < encoderStart + 4000) {
              left_drive.setPower(0.25);
              right_drive.setPower(0.25);
              return this;
          } else {
              left_drive.setPower(0);
              right_drive.setPower(0);
              return dropGlyph;
          }
        }
        private int encoderStart;
    }

    /**
     * Drops glyph.
     */
    public class DropGlyph implements StateMachine.State {
        @Override
        public void start() {
                left_thumb.setPosition(0);
                right_thumb.setPosition(0);
        }

        @Override
        public State update() {

            return backUpForArm;

        }
    }

    /**
     * Backs up enough for arm to raise.
     */
    public class BackUpForArm implements StateMachine.State {
        @Override
        public void start() {
                encoderStart = left_drive.getCurrentPosition();
        }

        @Override
        public State update() {
            if(left_drive.getCurrentPosition() > encoderStart - 1000){
                    left_drive.setPower(-0.25);
                    right_drive.setPower(-0.25);
                    return this;
            } else {
                    left_drive.setPower(0);
                    right_drive.setPower(0);
                    return dropArm;
            }
        }
        private int encoderStart;
    }

    /**
     * Drops arm.
     */
    public class DropArm implements StateMachine.State {
        @Override
        public void start() {
        }

        @Override
        public State update() {
            if(mr_gyro.getHeading() > 1){
                    arm_lift.setPower(0.25);
                    return this;
            } else {
                    arm_lift.setPower(0);
                    return shoveGlyphIn;
            }
        }
    }

    /**
     * Shoves glyph in.
     */
    public class ShoveGlyphIn implements StateMachine.State {
        @Override
        public void start() {
                encoderStart = left_drive.getCurrentPosition();
        }

        @Override
        public State update() {
                if (left_drive.getCurrentPosition() < encoderStart + 4000) {
                    left_drive.setPower(0.25);
                    right_drive.setPower(0.25);
                    return this;
                } else {
                    left_drive.setPower(0);
                    right_drive.setPower(0);
                    return finalBackUp;
                }
        }
        private int encoderStart;
    }

    /**
     * Backs up a little bit so glyph is not touching bot.
     */
    public class FinalBackUp implements StateMachine.State {
        @Override
        public void start() {
                encoderStart = left_drive.getCurrentPosition();
        }

        @Override
        public State update() {
                if(left_drive.getCurrentPosition() > encoderStart - 500){
                        left_drive.setPower(-0.25);
                        right_drive.setPower(-0.25);
                        return this;
                } else {
                        left_drive.setPower(0);
                        right_drive.setPower(0);
                        return null;
                }
        }
        private int encoderStart;
    }

     @Override

     public void runOpMode(){

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
         machine = new StateMachine(pickUpGlyph);

         waitForStart();

         while(opModeIsActive()){
             machine.update();
         }

       }

    // Hardware stuff.
    private static DcMotor left_drive;
    private static DcMotor right_drive;
    private static DcMotor arm_lift;
    private static ColorSensor color_prox;
    private static Servo right_thumb;
    private static Servo left_thumb;
    private static Servo ball_arm;
    private GyroSensor mr_gyro;
    static BNO055IMU imu;

    // The state machine manager.
    private StateMachine machine;
    // Pick up starting glyph
    private PickUpGlyph pickUpGlyph;
    // Bring color sensor arm down
    private LowerColorSensor lowerColorSensor;
    // Knocks red ball off
    private KnockRedBallOff knockRedBallOff;
    // Rotates unitl certain angle
    private RotateUntilAngle rotateUntilAngle;
    // Drives forward putting glyph in slot
    private DriveGlyphToSlot driveGlyphToSlot;
    // Releases glyph
    private DropGlyph dropGlyph;
    // Backs up for arm
    private BackUpForArm backUpForArm;
    // Drops arm.
    private DropArm dropArm;
    // Shoves glyph in
    private ShoveGlyphIn shoveGlyphIn;
    // Final back up
    private FinalBackUp finalBackUp;
}
