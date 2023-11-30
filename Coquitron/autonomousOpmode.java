package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.openCv.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//NAVX IMU Integration
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;

@Autonomous(name = "AutonomousV222_WIP")
public class autonomousOpmode extends LinearOpMode {

    private DcMotor sliderMotor;
    private BNO055IMU imu;
//    private VuforiaCurrentGame vuforiaPOWERPLAY;
//    private Tfod tfod;
    private DcMotor left_back;
    private DcMotor left_front;
    private DcMotor right_back;
    private DcMotor right_front;
    private Servo Gripper;

    // NAVX IMU Integration variables
    private AHRS navx_device;
    private ElapsedTime runtime = new ElapsedTime();
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private boolean calibration_complete = false;
    navXPIDController.PIDResult yawPIDResult;
    // NAV IMU Integration variables end

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
    int getParkingSpot;
//    ElapsedTime runtime;
    ElapsedTime cameraTimer;
    double initialTime;

    /**
     * Sets the arm position
     */
//    private void setSliderPosition(double sliderPosition) {
//        boolean gripperOriginalState;
//
//        gripperOriginalState = gripperOn;
//        setGripperState(true);
//        // Is this needed?
//        sleep(250);
//        sliderMotor.setTargetPosition((int) (sliderPosition * sliderTicksPerInch));
//        ((DcMotorEx) sliderMotor).setVelocity(4000);
//        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        gripperOn = gripperOriginalState;
//        sleep(250);
//    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        vuforiaPOWERPLAY = new VuforiaCurrentGame();
//        tfod = new Tfod();
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        Gripper = hardwareMap.get(Servo.class, "Gripper");

//        initializeIMU();
        initializeMotors();
        initializeNavX();
        initializeVariables();
//        initCamera();
        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
        waitForStart();
        if (opModeIsActive()) {
            main();
        }

//        vuforiaPOWERPLAY.close();
//        tfod.close();
    }

    public void initializeNavX() {
        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (calibration_complete) {
                navx_device.zeroYaw();
            } else {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
    }

    /**
     * Initialize the IMU
     */
//    private void initializeIMU() {
//        BNO055IMU.Parameters imuParameters;
//        String message;
//
//        // Create new IMU Parameters object.
//        imuParameters = new BNO055IMU.Parameters();
//        // Use degrees as angle unit.
//        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        // Express acceleration as m/s^2.
//        // Disable logging.
//        imuParameters.loggingEnabled = false;
//        imu.initialize(imuParameters);
//        message = "";
//        while (!imu.isGyroCalibrated()) {
//            message = message + ".";
//            telemetry.addData("Calibrating Gyro:", message);
//            telemetry.update();
//        }
//        telemetry.addData("Calibration:", "Done!");
//        telemetry.update();
//    }

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
        double d=5;
//        setGripperState(false);
        moveDistance(10, 1,8,false,0);
        rotateAngleGlobal(180,1,1);
//        rotateAngleGlobal(180,10,5);
        moveDistance(50,1,8,false,0);
        moveDistance(-20,1,8,false,0);
//        setSliderPosition(14);
//        setGripperState(true);
//        strafeDistance(-6,1);
//        setSliderPosition(14);

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
     * Describe this function...
     */
    private double calculateError(double desiredAngle, double currentAngle) {
        double angleError;

//        angleError = desiredAngle - currentAngle;
        angleError = desiredAngle + currentAngle;
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
     * Sets the gripper state
//     */
//    private void setGripperState(boolean isGripperOn) {
//        gripperOn = !isGripperOn;
//        if (gripperOn) {
//            Gripper.setPosition(0);
//        } else {
//            Gripper.setPosition(1);
//        }
//    }

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

//    private int initCamera() {
//        OpenCvCamera camera;
//        AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
////        double FEET_PER_METER = 3.28084;
//
//        // Lens intrinsics
//        // UNITS ARE PIXELS
//        // Calibration might be needed for other configurations
//        double fx = 578.272;
//        double fy = 578.272;
//        double cx = 402.145;
//        double cy = 221.506;
//
//        // UNITS ARE METERS
//        double tagsize = 0.166;
//
//        // Tad IDs
//        int Left = 5;
//        int Middle = 8;
//        int Right = 11;
////        int position = 0;
//        AprilTagDetection tagOfInterest = null;
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//        cameraTimer = new ElapsedTime();
//        initialTime = cameraTimer.seconds();
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        while (!isStarted() && !isStopRequested() && (cameraTimer.seconds() <= initialTime + 5))
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    for(AprilTagDetection tag : currentDetections) {
//                        if (tag.id == Left) {
//                            telemetry.addLine("Found cone tag 1!\n\nLocation data:");
//                            tagToTelemetry(tagOfInterest);
//                            getParkingSpot = 1;
//                        } else if (tag.id == Middle) {
//                            telemetry.addLine("Found cone tag 2!\n\nLocation data:");
//                            tagToTelemetry(tagOfInterest);
//                            getParkingSpot = 2;
//                        } else if (tag.id == Right) {
//                            telemetry.addLine("Found cone tag 3!\n\nLocation data:");
//                            tagToTelemetry(tagOfInterest);
//                            getParkingSpot =3;
//                        }
//                    }
////                    telemetry.addLine("Found tag of interest...\n\nLocation data:");
////                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("No tag of interest has been found");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("Tag has never been seen...");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nTag previously seen before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("No tag of interest found");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("The tag has never been seen...");
//                }
//                else
//                {
//                    telemetry.addLine("\nTag previously seen before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available");
//            telemetry.update();
//        }
//        if(tagOfInterest == null)
//        {
//            getParkingSpot = 3;
//        }
//
//        return getParkingSpot;
//
//
//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
////        while (opModeIsActive()) {sleep(20);}
//    }

//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        final double FEET_PER_METER = 3.28084;
//        telemetry.addLine(String.format("\nDetected QR tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }

    /**
     * Get current angle from IMU
     */
    private double getAngle() {
//        Orientation imuAngles;
        float finalAngle;

        finalAngle = navx_device.getYaw();

//        imuAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        finalAngle = imuAngles.firstAngle;
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
    public void moveDistance(double distance,
                             double speed,
                             double timeout,
                             boolean useGyro,
                             double heading
    ){


        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;
        runtime = new ElapsedTime();


        // The potentially adjusted current target heading
        double curHeading = heading;


        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp


        double leftDistance = distance;
        double rightDistance = distance;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop


            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 14 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += 2 * 3.1415 * 14 * headingChange / 360.0;
                    } else {
                        // Assume 16 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= 2 * 3.1415 * 14 * headingChange / 360.0;
                    }
                }
            }


            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = left_front.getCurrentPosition() + (int) (leftDistance * inchesToTicks);
            newLRTarget = left_back.getCurrentPosition() + (int) (leftDistance * inchesToTicks);
            newRFTarget = right_front.getCurrentPosition() + (int) (rightDistance * inchesToTicks);
            newRRTarget = right_back.getCurrentPosition() + (int) (rightDistance * inchesToTicks);


            left_front.setTargetPosition(newLFTarget);
            right_front.setTargetPosition(newRFTarget);
            left_back.setTargetPosition(newLRTarget);
            right_back.setTargetPosition(newRRTarget);


            // Turn On motors to RUN_TO_POSITION
            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();


            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED, speed);


            // Set the motors to the starting power
            left_front.setPower(Math.abs(curSpeed));
            right_front.setPower(Math.abs(curSpeed));
            left_back.setPower(Math.abs(curSpeed));
            right_back.setPower(Math.abs(curSpeed));


            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    left_front.isBusy() &&
                    left_back.isBusy() &&
                    right_front.isBusy() &&
                    right_back.isBusy()) {


                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;


                // Doing gyro heading correction?
                if (useGyro) {


                    // adjust relative speed based on heading
                    double error = getError(curHeading);
                    double steer = getSteer(error, 0.05);


                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;


                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;


                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }


                }


                // And rewrite the motor speeds
                left_front.setPower(Math.abs(leftSpeed));
                right_front.setPower(Math.abs(rightSpeed));
                left_back.setPower(Math.abs(leftSpeed));
                right_back.setPower(Math.abs(rightSpeed));


                // Allow time for other processes to run.
                idle();
            }


            // Stop all motion;
            left_front.setPower(0);
            right_front.setPower(0);
            left_back.setPower(0);
            right_back.setPower(0);


            // Turn off RUN_TO_POSITION
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */


    public double getError(double targetAngle) {


        double robotError;


        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */


    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
