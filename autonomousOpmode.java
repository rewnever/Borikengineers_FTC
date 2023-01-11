package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "AutonomousV222")
public class autonomousOpmode extends LinearOpMode {

    private DcMotor sliderMotor;
    private BNO055IMU imu;
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private DcMotor left_back;
    private DcMotor left_front;
    private DcMotor right_back;
    private DcMotor right_front;
    private Servo Gripper;

    double angleSetPoint;
    double fwdBack;
    boolean gripperOn;
    int sliderSetpoint;
    double turn;
    int strafe;
    double leftFrontPower;
    Recognition recognition;
    double angleCurrentError;
    double inchesToTicks;
    int sliderTicksPerInch;
    double error;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /**
     * Sets the arm position
     */
    private void setSliderPosition(double sliderPosition) {
        boolean gripperOriginalState;

        gripperOriginalState = gripperOn;
        setGripperState(true);
        // Is this needed?
        sleep(250);
        sliderMotor.setTargetPosition((int) (sliderPosition * sliderTicksPerInch));
        ((DcMotorEx) sliderMotor).setVelocity(4000);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperOn = gripperOriginalState;
        sleep(250);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        Gripper = hardwareMap.get(Servo.class, "Gripper");

        initializeIMU();
        initializeMotors();
        initializeVariables();
        waitForStart();
        if (opModeIsActive()) {
            main();
        }

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    /**
     * Initialize the IMU
     */
    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;
        String message;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        // Disable logging.
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        message = "";
        while (!imu.isGyroCalibrated()) {
            message = message + ".";
            telemetry.addData("Calibrating Gyro:", message);
            telemetry.update();
        }
        telemetry.addData("Calibration:", "Done!");
        telemetry.update();
    }

    /**
     * Rotates the robot using the IMU
     */
    private void rotateAngleGlobal(int angle,double anglePower, double angleTolerance) {
        angleSetPoint = angle;
        anglePower = Math.abs(anglePower);
        fwdBack = 0;
        strafe = 0;
        angleCurrentError = calculateError(angleSetPoint, getAngle());
        while (Math.abs(angleCurrentError) > angleTolerance && opModeIsActive()) {
            angleCurrentError = calculateError(angleSetPoint, getAngle());
            turn = anglePower * -(angleCurrentError / 30);
            if (turn > 0 && turn < 0.2) {
                turn = 0.2;
            } else if (turn > -0.2 && turn < 0) {
                turn = -0.2;
            }
            telemetry.addData("error", angleCurrentError);
            telemetry.addData("turn", turn);
            applyMotorPower();
            doTelemetry();
        }
        stopMotors();
    }

    /**
     * Initialize motors
     */
    private void initializeMotors() {
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Servo limits set from 0.09 to 0.45. This servo responds to a
        // minimum value of 0.09. Maximum value is set to physical limit.
        Gripper.scaleRange(0.23, 0.6);
    }

    /**
     * Describe this function...
     */
    private void test() {
        List<Recognition> recognitions;
        int index;

        while (opModeIsActive()) {
            recognitions = tfod.getRecognitions();
            if (JavaUtil.listLength(recognitions) == 0) {
                telemetry.addData("nothin ", "(");
            } else {
                index = 0;
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    displayInfo(index);
                    index += 1;
                }
            }
            telemetry.update();
        }
    }

    /**
     * Sets various parameters used in the code
     */
    private void initializeVariables() {
        int ticksPerRevolution;
        int wheelDiameter;
        int gearRatio;
        ElapsedTime barCodeTimer;
        ElapsedTime armTimer;
        ElapsedTime gripperTimer;
        double gripperOpen;
        double gripperClosed;

        // MotorVariables
        ticksPerRevolution = 960;
        wheelDiameter = 4;
        gearRatio = 1;
        inchesToTicks = (gearRatio * ticksPerRevolution) / (Math.PI * wheelDiameter);
        // Timer Variables
        barCodeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        barCodeTimer.reset();
        armTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        armTimer.reset();
        gripperTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        gripperTimer.reset();
        // Motion Variables
        strafe = 0;
        turn = 0;
        fwdBack = 0;
        // Slider Variables
        sliderSetpoint = 0;
        // Gripper Variables
        gripperOn = false;
        gripperOpen = 0.23;
        gripperClosed = 0.6;
        sliderTicksPerInch = 100;
    }

    /**
     * Autonomous functions for blue alliance near the carrousel
     */
    private void main() {
        setGripperState(false);
        moveDistance(3, 1);
        setSliderPosition(14);
        setGripperState(true);
        setSliderPosition(14);
        rotateAngleGlobal(38, 1, 1);
        moveDistance(9, 1);
        while (right_front.isBusy()) {
        }
        setGripperState(false);
        sleep(300);
        moveDistance(-9, 1);
        setGripperState(true);
        setSliderPosition(0);
        rotateAngleGlobal(0, 1, 0.5);
        setGripperState(false);
        moveDistance(56, 1);
        moveDistance(-5, 1);
        setSliderPosition(5);
        setGripperState(false);
        rotateAngleGlobal(-90, 1, 1);
        setGripperState(false);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        moveDistance( 23.3, 1);
        moveDistance(-1, 1);
        setGripperState(true);
        setSliderPosition(14.5);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveDistance(-6, 1);
        rotateAngleGlobal(150, 1, 1);
        moveDistance(5.5F, 1);
        setGripperState(false);
        moveDistance(-5.5, 1);
        setSliderPosition(3.25);
        setGripperState(false);
        rotateAngleGlobal(-90, 1, 1);
        moveDistance(6, 1);
        setSliderPosition(14.5);
        moveDistance(-6, 1);
        setGripperState(true);
        rotateAngleGlobal(150, 1, 1);
        moveDistance(5.5F, 1);
        setGripperState(false);
        sleep(300);
        moveDistance(-5.5, 1);
        rotateAngleGlobal(90, 1, 1);
        moveDistance(48, 1);
    }

    /**
     * Applies power to the motors
     */
    private void applyMotorPower() {
        if (opModeIsActive()) {
            left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontPower = fwdBack + turn;
            leftFrontPower = leftFrontPower + strafe;
            rightFrontPower = fwdBack - turn;
            rightFrontPower = rightFrontPower - strafe;
            leftBackPower = fwdBack + turn;
            leftBackPower = leftBackPower - strafe;
            rightBackPower = fwdBack - turn;
            rightBackPower = rightBackPower + strafe;
            left_front.setPower(leftFrontPower);
            left_back.setPower(leftBackPower);
            right_front.setPower(rightFrontPower);
            right_back.setPower(rightBackPower);
        }
    }

    /**
     * Rotates the robot using the IMU
     */
    private void rotateAngle(double angle, double anglePower,
                             // TODO: Enter the type for argument named angleTolerance
                             double angleTolerance) {
        double rotP;

        rotP = 0.03;
        angleSetPoint = getAngle() + angle;
        if (angle < 0) {
            anglePower = -1 * Math.abs(anglePower);
        } else {
            anglePower = Math.abs(anglePower);
        }
        if (angleSetPoint > 180) {
            angleSetPoint = angleSetPoint - 360;
        } else if (angleSetPoint < -180) {
            angleSetPoint = angleSetPoint + 360;
        }
        fwdBack = 0;
        strafe = 0;
        angleCurrentError = angleSetPoint - getAngle();
        while (Math.abs(angleCurrentError) > angleTolerance && opModeIsActive()) {
            angleCurrentError = angleSetPoint - getAngle();
            if (angleCurrentError < 0 && angleCurrentError > -180) {
                // Turning left is more efficient in this situation
                turn = anglePower;
            } else if (angleCurrentError < -180) {
                // Turning right is more efficient in this situation
                turn = -anglePower;
            } else if (angleCurrentError > 0 && angleCurrentError < 180) {
                // Turning right is more efficient in this situation
                turn = -anglePower;
            } else if (angleCurrentError > 180) {
                // Turning left is more efficient in this situation
                turn = anglePower;
            } else {
                turn = 0;
            }
            if (angleCurrentError > 180) {
                angleCurrentError = angleCurrentError - 360;
            } else if (angleCurrentError < -180) {
                angleCurrentError = angleCurrentError + 360;
            }
            turn = turn * angleCurrentError * rotP;
            if (turn > 0 && turn < 0.3) {
                turn = 0.3;
            } else if (turn > -0.3 && turn < 0) {
                turn = -0.3;
            }
            applyMotorPower();
            doTelemetry();
        }
        stopMotors();
    }

    /**
     * Describe this function...
     */
    private double calculateError(double desiredAngle, double currentAngle) {
        double angleError;

        angleError = desiredAngle - currentAngle;
        if (angleError < -180) {
            angleError += 360;
        } else if (angleError > 180) {
            angleError += -360;
        }
        return angleError;
    }

    /**
     * Strafe left and right to distance
     */
    private void strafeDistance(double distance, int power) {
        fwdBack = 0;
        turn = 0;
        strafe = power;
        leftFrontPower = fwdBack + turn;
        leftFrontPower = leftFrontPower + strafe;
        rightFrontPower = fwdBack - turn;
        rightFrontPower = rightFrontPower - strafe;
        leftBackPower = fwdBack + turn;
        leftBackPower = leftBackPower - strafe;
        rightBackPower = fwdBack - turn;
        rightBackPower = rightBackPower + strafe;
        telemetry.addData("fwdBack", fwdBack);
        telemetry.update();
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setTargetPosition((int) ((distance * inchesToTicks) / 1));
        left_back.setTargetPosition((int) (-(distance * inchesToTicks) / 1));
        right_front.setTargetPosition((int) (-(distance * inchesToTicks) / 1));
        right_back.setTargetPosition((int) ((distance * inchesToTicks) / 1));
        left_front.setPower(leftFrontPower);
        left_back.setPower(leftBackPower);
        right_front.setPower(rightFrontPower);
        right_back.setPower(rightBackPower);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ((left_front.isBusy() || left_back.isBusy() || right_front.isBusy() && right_front.isBusy()) && opModeIsActive()) {
        }
        sleep(250);
    }

    /**
     * Move forward or back to specified distance
     */
    private void moveDistance(double distance, double power) {
        double angle;
        double initialAngle;

        initialAngle = getAngle();
        if (distance < 0) {
            power = -1 * Math.abs(power);
        } else {
            power = Math.abs(power);
        }
        fwdBack = power;
        turn = 0;
        strafe = 0;
        leftFrontPower = fwdBack + turn;
        leftFrontPower = leftFrontPower + strafe;
        rightFrontPower = fwdBack - turn;
        rightFrontPower = rightFrontPower - strafe;
        leftBackPower = fwdBack + turn;
        leftBackPower = leftBackPower - strafe;
        rightBackPower = fwdBack - turn;
        rightBackPower = rightBackPower + strafe;
        telemetry.addData("fwdBack", fwdBack);
        telemetry.update();
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setTargetPosition((int) (distance * inchesToTicks));
        left_back.setTargetPosition((int) (distance * inchesToTicks));
        right_front.setTargetPosition((int) (distance * inchesToTicks));
        right_back.setTargetPosition((int) (distance * inchesToTicks));
        left_front.setPower(leftFrontPower);
        left_back.setPower(leftBackPower);
        right_front.setPower(rightFrontPower);
        right_back.setPower(rightBackPower);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleSetPoint = getAngle();
        while (left_front.isBusy() && left_back.isBusy() && right_front.isBusy() && right_front.isBusy() && opModeIsActive()) {
            angle = getAngle();
            error = calculateError(angleSetPoint, angle);
            left_front.setPower(leftFrontPower - error / 90);
            left_back.setPower(leftBackPower - error / 90);
            right_front.setPower(rightFrontPower + error / 90);
            right_back.setPower(rightBackPower + error / 90);
            telemetry.addData("angleSetPoint", angleSetPoint);
            telemetry.addData("currentAngle", angle);
            telemetry.addData("un-normalized error", angleSetPoint - angle);
            telemetry.addData("error", error);
            telemetry.update();
        }
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    /**
     * Sets the gripper state
     */
    private void setGripperState(boolean isGripperOn) {
        gripperOn = !isGripperOn;
        if (gripperOn) {
            Gripper.setPosition(0);
        } else {
            Gripper.setPosition(1);
        }
    }

    /**
     * Stops the motors by settings its power to zero
     */
    private void stopMotors() {
        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
    }

    /**
     * Describe this function...
     */
    private int getArmPosition() {
        return sliderSetpoint;
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display lower corner info.
    }

    /**
     * Do all the telemetry
     */
    private void doTelemetry() {
        double armPower;
        boolean armMode;
//        double angleErrorDelta;

        telemetry.addData("Current Position", sliderMotor.getCurrentPosition());
        telemetry.addData("SetPoint", sliderSetpoint);
        telemetry.addData("angleCurrentError", angleCurrentError);
        telemetry.addData("currentAngle", getAngle());
        telemetry.addData("angleSetPoint", angleSetPoint);
//        telemetry.addData("angleErrorDelta", angleErrorDelta);
        if (turn > 0) {
            telemetry.addData("Robot turning", "Right");
        } else {
            telemetry.addData("Robot turning", "Left");
        }
        telemetry.update();
    }

    /**
     * Get current angle from IMU
     */
    private double getAngle() {
        Orientation imuAngles;
        float finalAngle;

        imuAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        finalAngle = imuAngles.firstAngle;
        return finalAngle;
    }

    /**
     * Returns the maximum absolute value of all the entered values.
     */
    private double absoluteMax(double a, double b, double c, double d) {
        double maxValue;

        maxValue = a;
        if (Math.abs(b) > maxValue) {
            maxValue = Math.abs(b);
        }
        if (Math.abs(c) > maxValue) {
            maxValue = Math.abs(c);
        }
        if (Math.abs(d) > maxValue) {
            maxValue = Math.abs(d);
        }
        return maxValue;
    }
}
