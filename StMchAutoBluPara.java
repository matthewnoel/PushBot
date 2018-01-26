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

public class StMchAutoBluPara extends LinearOpMode{
        /**
         * Calibrates gyro.
         */
        public class CalibrateGyro implements StateMachine.State {
            @Override
            public void start() {
                arm_gyro.calibrate();
                body_gyro.calibrate();
            }

            @Override
            public State update() {
                if(arm_gyro.isCalibrating() || body_gyro.isCalibrating()){
                        return this;
                } else {
                        return scanKey;
                }
            }
        }

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
                                    //telemetry.addData("Pose", format(pose));

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
                                return this;
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
                if(arm_gyro.getHeading() < 40){
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
                if(ball_arm.getPosition() < 0.5){
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
                    body_gyro.resetZAxisIntegrator();
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
                            if(body_gyro.getHeading() < 20){
                                    front_left.setPower(-0.125);
                                    back_left.setPower(-0.125);
                                    front_right.setPower(0.125);
                                    back_right.setPower(0.125);
                                    return this;
                            } else {

                                    front_left.setPower(0);
                                    back_left.setPower(0);
                                    front_right.setPower(0);
                                    back_right.setPower(0);
                                    ball_arm.setPosition(0);
                                    return rotateBack;
                            }
                    } else {
                            // Rotate right and knock off red ball.
                            if ((body_gyro.getHeading() > 340) || body_gyro.getHeading() == 0) {
                                    front_left.setPower(0.125);
                                    back_left.setPower(0.125);
                                    front_right.setPower(-0.125);
                                    back_right.setPower(-0.125);
                                    telemetry.addData("gyro", body_gyro.getHeading());
                                    telemetry.update();
                                    return this;
                            } else {
                                    front_left.setPower(0);
                                    back_left.setPower(0);
                                    front_right.setPower(0);
                                    back_right.setPower(0);
                                    ball_arm.setPosition(0);
                                    return rotateBack;
                            }
                    }
            }
        }

/**
   * Resets back to original rotation.
   */
  public class RotateBack implements StateMachine.State {
          @Override
          public void start() {
                  body_gyro.resetZAxisIntegrator();
          }

          @Override
          public State update() {
            if(!isLeft){
                            // Rotate left and knock off red ball.
                            if(body_gyro.getHeading() < 20){
                                    front_left.setPower(-0.125);
                                    back_left.setPower(-0.125);
                                    front_right.setPower(0.125);
                                    back_right.setPower(0.125);
                                    return this;
                            } else {

                                    front_left.setPower(0);
                                    back_left.setPower(0);
                                    front_right.setPower(0);
                                    back_right.setPower(0);
                                    ball_arm.setPosition(0);
                                    return rotateOnStone;
                            }
                    } else {
                            // Rotate right and knock off red ball.
                            if ((body_gyro.getHeading() > 340) || body_gyro.getHeading() == 0) {
                                    front_left.setPower(0.125);
                                    back_left.setPower(0.125);
                                    front_right.setPower(-0.125);
                                    back_right.setPower(-0.125);
                                    telemetry.addData("gyro", body_gyro.getHeading());
                                    telemetry.update();
                                    return this;
                            } else {
                                    front_left.setPower(0);
                                    back_left.setPower(0);
                                    front_right.setPower(0);
                                    back_right.setPower(0);
                                    ball_arm.setPosition(0);
                                    return rotateOnStone;
                            }
                    }
          }
  }

  /**
   * Rotates robot on balancing stone.
   */
  public class RotateOnStone implements StateMachine.State {
          @Override
          public void start() {
                  body_gyro.resetZAxisIntegrator();
          }

          @Override
          public State update() {
              if (body_gyro.getHeading() < 60) {
                      front_left.setPower(-0.25);
                      back_left.setPower(-0.25);
                      front_right.setPower(0.25);
                      back_right.setPower(0.25);
                      return this;
              } else {

                      front_left.setPower(0);
                      back_left.setPower(0);
                      front_right.setPower(0);
                      back_right.setPower(0);
                      return driveOffStone;
              }
          }
  }

  /**
   * Drive off balancing stone.
   */
  public class DriveOffStone implements StateMachine.State {
          @Override
          public void start() {
            encoderStart = back_left.getCurrentPosition();
          }

          @Override
          public State update() {
                  telemetry.addData("Position",glyphPosition);
                  telemetry.update();
            if (back_left.getCurrentPosition() < encoderStart + 5000) {
                back_left.setPower(0.25);
                front_left.setPower(0.25);
                back_right.setPower(0.25);
                front_right.setPower(0.25);
                return this;
            } else {
                    back_left.setPower(0);
                    front_left.setPower(0);
                    back_right.setPower(0);
                    front_right.setPower(0);
                if(glyphPosition.equals("LEFT")){
                        return rotateToLeft;
                } else if (glyphPosition.equals("RIGHT")){
                        return rotateToRight;
                } else {
                        return rotateToCenter;
                }
            }
          }
          private int encoderStart;
  }

  /**
   * Rotates to left column.
   */
  public class RotateToLeft implements StateMachine.State {
      @Override
      public void start() {
              body_gyro.resetZAxisIntegrator();
      }

      @Override
      public State update() {
          if ((body_gyro.getHeading() < 60)){
                  back_left.setPower(-0.25);
                  front_left.setPower(-0.25);
                  back_right.setPower(0.25);
                  front_right.setPower(0.25);
                  return this;
          } else {

                  back_left.setPower(0);
                  front_left.setPower(0);
                  back_right.setPower(0);
                  front_right.setPower(0);
                  return driveGlyphToSlot;
          }
      }
  }

  /**
   * Rotates to center column.
   */
  public class RotateToCenter implements StateMachine.State {
      @Override
      public void start() {
              body_gyro.resetZAxisIntegrator();
      }

      @Override
      public State update() {
          if ((body_gyro.getHeading() < 60)){
                  back_left.setPower(-0.25);
                  front_left.setPower(-0.25);
                  back_right.setPower(0.25);
                  front_right.setPower(0.25);
                  return this;
          } else {

                  back_left.setPower(0);
                  front_left.setPower(0);
                  back_right.setPower(0);
                  front_right.setPower(0);
                  return driveGlyphToSlot;
          }
      }
  }

  /**
   * Rotates to left column.
   */
  public class RotateToRight implements StateMachine.State {
      @Override
      public void start() {
              body_gyro.resetZAxisIntegrator();
      }

      @Override
      public State update() {
          if ((body_gyro.getHeading() < 60)){
                  back_left.setPower(-0.25);
                  front_left.setPower(-0.25);
                  back_right.setPower(0.25);
                  front_right.setPower(0.25);
                  return this;
          } else {

                  back_left.setPower(0);
                  front_left.setPower(0);
                  back_right.setPower(0);
                  front_right.setPower(0);
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
           encoderStart = back_left.getCurrentPosition();
         }

         @Override
         public State update() {
           if (back_left.getCurrentPosition() < encoderStart + 1000) {
               back_left.setPower(0.25);
               front_left.setPower(0.25);
               back_right.setPower(0.25);
               front_right.setPower(0.25);
               return this;
           } else {
                   back_left.setPower(0);
                   front_left.setPower(0);
                   back_right.setPower(0);
                   front_right.setPower(0);
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
                   encoderStart = back_left.getCurrentPosition();
           }

           @Override
           public State update() {
               if(back_left.getCurrentPosition() > encoderStart - 1000){
                       back_left.setPower(-0.25);
                       front_left.setPower(-0.25);
                       back_right.setPower(-0.25);
                       front_right.setPower(-0.25);
                       return this;
               } else {
                       back_left.setPower(0);
                       front_left.setPower(0);
                       back_right.setPower(0);
                       front_right.setPower(0);
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
               if(arm_gyro.getHeading() > 10){
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
                    encoderStart = back_left.getCurrentPosition();
            }

            @Override
            public State update() {
                    if (back_left.getCurrentPosition() < encoderStart + 4000) {
                        back_left.setPower(0.25);
                        front_left.setPower(0.25);
                        back_right.setPower(0.25);
                        front_right.setPower(0.25);
                        return this;
                    } else {
                            back_left.setPower(0);
                            front_left.setPower(0);
                            back_right.setPower(0);
                            front_right.setPower(0);
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
                     encoderStart = back_left.getCurrentPosition();
             }

             @Override
             public State update() {
                     if(back_left.getCurrentPosition() > encoderStart - 500){
                             back_left.setPower(-0.25);
                             front_left.setPower(-0.25);
                             back_right.setPower(-0.25);
                             front_right.setPower(-0.25);
                             return this;
                     } else {
                             back_left.setPower(0);
                             front_left.setPower(0);
                             back_right.setPower(0);
                             front_right.setPower(0);
                             return null;
                     }
             }
             private int encoderStart;
         }

     @Override

     public void runOpMode(){

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
         parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
         this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

         /**
          * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
          * in this data set: all three of the VuMarks in the game were created from this one template,
          * but differ in their instance id information.
          * @see VuMarkInstanceId
          */
         VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
         relicTemplate = relicTrackables.get(0);
         relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

         relicTrackables.activate();
 //END VUFORIA CODE

         arm_lift = hardwareMap.get(DcMotor.class, "arm_lift");
         front_right = hardwareMap.get(DcMotor.class, "front_right");
         front_left = hardwareMap.get(DcMotor.class, "front_left");
         back_right = hardwareMap.get(DcMotor.class, "back_right");
         back_left = hardwareMap.get(DcMotor.class, "back_left");
         color_prox = hardwareMap.get(ColorSensor.class, "color_prox");
         right_thumb = hardwareMap.get(Servo.class, "right_thumb");
         left_thumb = hardwareMap.get(Servo.class, "left_thumb");
         ball_arm = hardwareMap.get(Servo.class, "ball_arm");
         arm_gyro = hardwareMap.get(GyroSensor.class, "arm_gyro");
         body_gyro = hardwareMap.get(GyroSensor.class, "body_gyro");

         front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         arm_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         arm_lift.setDirection(DcMotor.Direction.REVERSE);
         //front_left.setDirection(DcMotor.Direction.REVERSE);
         front_right.setDirection(DcMotor.Direction.REVERSE);
         //back_left.setDirection(DcMotor.Direction.REVERSE);
         back_right.setDirection(DcMotor.Direction.REVERSE);

         calibrateGyro = new CalibrateGyro();
         scanKey = new ScanKey();
         pickUpGlyph = new PickUpGlyph();
         lowerColorSensor = new LowerColorSensor();
         knockRedBallOff = new KnockRedBallOff();
         rotateBack = new RotateBack();
         rotateOnStone = new RotateOnStone();
         driveOffStone = new DriveOffStone();
         rotateToLeft = new RotateToLeft();
         rotateToCenter = new RotateToCenter();
         rotateToRight = new RotateToRight();
         driveGlyphToSlot = new DriveGlyphToSlot();
         dropGlyph = new DropGlyph();
         backUpForArm =new BackUpForArm();
         dropArm = new DropArm();
         shoveGlyphIn = new ShoveGlyphIn();
         finalBackUp = new FinalBackUp();

         // Start the state machine with forward state.
         machine = new StateMachine(calibrateGyro);

         waitForStart();

         while(opModeIsActive()){
             machine.update();
         }

       }

    // Hardware stuff.
    private DcMotor arm_lift;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private ColorSensor color_prox;
    private Servo right_thumb;
    private Servo left_thumb;
    private Servo ball_arm;
    private GyroSensor arm_gyro;
    private GyroSensor body_gyro;
    private String glyphPosition;
    private boolean isLeft;

    private VuforiaTrackable relicTemplate;


    // The state machine manager.
    private StateMachine machine;
    //Calibrates gyro.
    private CalibrateGyro calibrateGyro;
    // Scan glyph.
    private ScanKey scanKey;
    // Pick up starting glyph
    private PickUpGlyph pickUpGlyph;
    // Bring color sensor arm down
    private LowerColorSensor lowerColorSensor;
    // Knocks red ball off
    private KnockRedBallOff knockRedBallOff;
    // Resets rotation after knocking off ball.
    private RotateBack rotateBack;
    // Rotates robot on stone.
    private RotateOnStone rotateOnStone;
    // Drives off stone.
    private DriveOffStone driveOffStone;
    // Rotates until left column.
    private RotateToLeft rotateToLeft;
    // Rotates until center column.
    private RotateToCenter rotateToCenter;
    // Rotates until right column.
    private RotateToRight rotateToRight;
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


    //START VUFORIA CODE
        public static final String TAG = "Vuforia VuMark Sample";

        OpenGLMatrix lastLocation = null;

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;
    //END VUFORIA CODE
}
