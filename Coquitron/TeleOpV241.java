package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "TeleOpV241_demo")
public class TeleOpV241 extends LinearOpMode {

     private DcMotor sliderMotor;
    // private Servo Gripper;
    // private TouchSensor armTouchSensor;
//    private DcMotor right_front;
//    private DcMotor left_front;
    private DcMotor right_back;
    private DcMotor left_back;
//    private ColorSensor colorSensor_REV_ColorRangeSensor;

     int sliderSetPoint;
    // ElapsedTime gripperTimer;
    float drive;
     boolean armMode;
     boolean pArmMode;
    // boolean gripperOn;
    // double gripperClosed;
    float strafe;
    double turn;
    // double gripperOpen;
    double denominator;
     int maxPosition;
     int lowPosition;
     int midPosition;
     double sliderSpeed;
     int sliderTolerance;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
         sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        // Gripper = hardwareMap.get(Servo.class, "Gripper");
        // armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouchSensor");
//        right_front = hardwareMap.get(DcMotor.class, "right_front");
//        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
//        colorSensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        initializeVariables();
        initializeMotors();
        waitForStart();
        if (opModeIsActive()) {
//             resetSlider();
            while (opModeIsActive()) {
                moveWheels();
                 setArmMode();
                // moveGripper();
                doTelemetry();
            }
        }
    }

    /**
     * Describe this function...
     */
     private void setArmMode() {
       if (gamepad1.a && !pArmMode) {
         armMode = !armMode;
       }
       pArmMode = gamepad1.a;
       sliderMotor.setTargetPosition(sliderMotor.getCurrentPosition());
       if (armMode) {
         moveSlideStepwise();
       } else {
         moveSliderContinuous();
       }
     }

    /**
     * Describe this function...
     */
    // private void moveGripper() {
    //   boolean previousGripperState;

    //   if (gamepad1.x && !previousGripperState) {
    //     gripperOn = !gripperOn;
    //   }
    //   // Gripper limits are set in the motor initialization routine
    //   if (gripperOn) {
    //     Gripper.setPosition(0);
    //   } else {
    //     Gripper.setPosition(0.5);
    //   }
    //   previousGripperState = gamepad1.x;
    // }

    /**
     * Describe this function...
     */
    // private void moveGripper2() {
    //   if (gamepad1.x && gripperTimer.milliseconds() >= 200) {
    //     gripperOn = !gripperOn;
    //     gripperTimer.reset();
    //   }
    //   // Gripper limits are set in the motor initialization routine
    //   if (gripperOn) {
    //     Gripper.setPosition(0);
    //   } else {
    //     Gripper.setPosition(0.5);
    //   }
    // }

    /**
     * Describe this function...
     */
     private void resetSlider() {
       ElapsedTime sliderTimer;

//     Gripper.setPosition(gripperClosed);
     sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     sliderMotor.setTargetPosition(0);
     sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     sliderMotor.setTargetPosition(500);
     sliderMotor.setPower(0.5);
     while (sliderMotor.isBusy()) {
       doTelemetry();
     }
     sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     sliderMotor.setPower(-0.25);
     sliderTimer = new ElapsedTime();
     // Continue moving until the button is pressed or 1 second has elapsed (in case the button is broken) or mechanism is stuck for any reason. This prevents the motor from burning due to it never reaching the zero position.
     while (sliderTimer.seconds() < 3) {
       doTelemetry();
       sliderTimer.log("time");
     }
       sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       sliderMotor.setTargetPosition(0);
       sliderMotor.setPower(0);
       doTelemetry();
     }

    /**
     * Describe this function...
     */
    private void initializeVariables() {
        // Timer used to prevent multiple consecutive button presses.
        // gripperTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // gripperTimer.reset();
        // gripperOn = false;
        // gripperOpen = 0.23;
        // gripperClosed = 0.6;
         maxPosition = 2750;
         midPosition = 2375;
         lowPosition = 1450;
         sliderSetPoint = sliderMotor.getCurrentPosition();
        denominator = 1;
         sliderSpeed = 0;
         sliderTolerance = 100;
           pArmMode = false;
           armMode = false;
    }

    /**
     * Describe this function...
     */
    private void initializeMotors() {
//        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.FORWARD);
         sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Servo limits set from 0.09 to 0.45. This servo responds to a
        // minimum value of 0.09. Maximum value is set to physical limit.
        // Gripper.scaleRange(gripperOpen, gripperClosed);
    }

    /**
     * Describe this function...
     */
    private void doTelemetry() {
        telemetry.addData("", "====WHEELS====");
//        telemetry.addData("LF /RF", left_front.getPower() + " / " + right_front.getPower());
        telemetry.addData("LF /RF", left_back.getPower() + " / " + right_back.getPower());
        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("LX", gamepad1.left_stick_x);
        telemetry.addData("LY", gamepad1.left_stick_y);
        telemetry.addData("RX", gamepad1.right_stick_x);
        telemetry.addData("", "");
         telemetry.addData("", "====SLIDER====");
         telemetry.addData("Current / Target", sliderMotor.getCurrentPosition() + " / " + sliderMotor.getTargetPosition());
         telemetry.addData("Speed", sliderSpeed);
         if (gamepad1.right_trigger > 0) {
           telemetry.addData("Status", "Going up");
         } else if (gamepad1.left_trigger > 0) {
           telemetry.addData("Status", "Going Down");
         } else {
           telemetry.addData("Status", "Stopped");
         }
        // if (armTouchSensor.isPressed()) {
        //   telemetry.addData("Limit Button", "Pressed");
        // } else {
        //   telemetry.addData("Limit Button", "Not Pressed");
        // }
         if (armMode) {
           telemetry.addData("Slider Mode", "Stepped");
         } else {
           telemetry.addData("Slider Mode", "Continuous");
         }
        telemetry.addData("", "");
//        telemetry.addData("", "====COLOR SENSOR====");
//        telemetry.addData("RGB", colorSensor_REV_ColorRangeSensor.red() + ", " + colorSensor_REV_ColorRangeSensor.green() + ", " + colorSensor_REV_ColorRangeSensor.blue());
        telemetry.addData("", "");
        // telemetry.addData("", "====GRIPPER====");
        // telemetry.addData("Gripper On", gripperOn);
        // telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void moveWheels() {
        double rightFrontPower;
        double leftFrontPower;
        double rightBackPower;
        double leftBackPower;

        drive = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x * -1.4;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(drive) + Math.abs(strafe) + Math.abs(turn)));
        rightFrontPower = JavaUtil.sumOfList(JavaUtil.createListWith(drive, -strafe, -turn)) / denominator;
        leftFrontPower = JavaUtil.sumOfList(JavaUtil.createListWith(drive, strafe, turn)) / denominator;
        rightBackPower = JavaUtil.sumOfList(JavaUtil.createListWith(drive, strafe, -turn)) / denominator;
        leftBackPower = JavaUtil.sumOfList(JavaUtil.createListWith(drive, -strafe, turn)) / denominator;
//        right_front.setPower(rightFrontPower);
//        left_front.setPower(leftFrontPower);
        right_back.setPower(rightBackPower);
        left_back.setPower(leftBackPower);
    }

    /**
     * Describe this function...
     */
     private void moveSliderContinuous() {
       sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       if (gamepad1.right_trigger > 0 && sliderMotor.getCurrentPosition() < maxPosition - 50) {
         sliderMotor.setPower(0.75);
       } else if (gamepad1.left_trigger > 0 && sliderMotor.getCurrentPosition() > 100) {
         sliderMotor.setPower(-0.75);
       } else {
         sliderMotor.setPower(0.001);
       }
     }

    /**
     * Describe this function...
     */
     private void moveSlideStepwise() {
       sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       if (gamepad1.right_trigger > 0) {
         if (sliderMotor.getCurrentPosition() < lowPosition - sliderTolerance) {
           sliderMotor.setTargetPosition(lowPosition);
         } else if (sliderMotor.getCurrentPosition() > lowPosition - sliderTolerance && sliderMotor.getCurrentPosition() < midPosition - 50) {
           sliderMotor.setTargetPosition(midPosition);
         } else if (sliderMotor.getCurrentPosition() > midPosition - sliderTolerance) {
           sliderMotor.setTargetPosition(maxPosition);
         }
       ((DcMotorEx) sliderMotor).setVelocity(10000);
     } else if (gamepad1.left_trigger > 0) {
       if (sliderMotor.getCurrentPosition() > midPosition + sliderTolerance) {
         sliderMotor.setTargetPosition(midPosition);
       } else if (sliderMotor.getCurrentPosition() > lowPosition + sliderTolerance && sliderMotor.getCurrentPosition() < midPosition + 200) {
         sliderMotor.setTargetPosition(lowPosition);
       } else if (sliderMotor.getCurrentPosition() < lowPosition + sliderTolerance) {
         sliderMotor.setTargetPosition(0);
       }
       ((DcMotorEx) sliderMotor).setVelocity(2000);
     }
   }
}
