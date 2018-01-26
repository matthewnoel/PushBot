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
         * Scans the pictograph.
         */
        public class ScanKey implements StateMachine.State {
            @Override
            public void start() {

            }

            @Override
            public State update() {
                    if(glyphPosition == null){
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
                    } else {
                        return pickUpGlyph;
                    }
              }
        }

  /**
   * Picks up glyph.
   */
  public class PickUpGlyph implements StateMachine.State {
      @Override
      public void start() {

      }

      @Override
      public State update() {
          left_thumb.setPosition(0.5);
          right_thumb.setPosition(0.5);
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
                              ball_arm.setPosition(0);
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
                              ball_arm.setPosition(0);
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
            if(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 68){
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
                right_thumb.setPosition(1);
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
            if(mr_gyro.getHeading() > 10){
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

         pickUpGlyph = new PickUpGlyph();
         lowerColorSensor = new LowerColorSensor();
         knockRedBallOff = new KnockRedBallOff();
         rotateUntilAngle = new RotateUntilAngle();
         driveGlyphToSlot = new DriveGlyphToSlot();
         dropGlyph = new DropGlyph();
         backUpForArm =new BackUpForArm();
         dropArm = new DropArm();
         shoveGlyphIn = new ShoveGlyphIn();
         finalBackUp = new FinalBackUp();

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
