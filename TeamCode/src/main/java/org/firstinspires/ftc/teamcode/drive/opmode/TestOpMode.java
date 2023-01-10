package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "SimpleTestOpMode", group = "Linear Opmode")

public class TestOpMode extends LinearOpMode {

    final double JOYSTICK_DEAD_ZONE = 0.20;
    final double JOYSTICK_MOVEMENT_SENSITIVITY = 0.75;
    final double JOYSTICK_ROTATION_SENSITIVITY = 1.00;

    final double ELEVATOR_HEIGHT_MAX = 4157;
    final double ELEVATOR_HEIGHT_LOW = 1760;
    final double ELEVATOR_HEIGHT_MIDDLE = 2860;
    final double ELEVATOR_HEIGHT_HIGH = 3990;
    final double ELEVATOR_HEIGHT_SHORT = 978;
    final double ELEVATOR_HEIGHT_BOTTOM = 297;
    final double ELEVATOR_ADJUST_RATE = 15.0;
    final double ELEVATOR_QUICK_ADJUST_HIGH_DOWN = 3897;
    final double ELEVATOR_QUICK_ADJUST_MIDDLE = 2705;
    final double ELEVATOR_QUICK_ADJUST_LOW = 1573;

    private final ElapsedTime runtime = new ElapsedTime();
    Orientation angles;
    Acceleration gravity;
    private double targetElevatorPosition = 0;
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftBackDriveMotor = null;
    private DcMotor rightBackDriveMotor = null;
    private DcMotor elevatorHeightControlMotor = null;
    private Servo intakeControlServo = null;
    private DigitalChannel limitSwitch = null;
    private DcMotor middleOdometry = null;
    private DcMotor rightOdometry = null;
    private DcMotor leftOdometry = null;
    private BNO055IMU imu;
    private RevBlinkinLedDriver revBlinkin = null;
    private ColorSensor leftLineFollower = null;
    private ColorSensor rightLineFollower = null;
    private DistanceSensor lineBreakSensor = null;




    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftRearDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightRearDriveMotor");
        elevatorHeightControlMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        intakeControlServo = hardwareMap.get(Servo.class, "intakeServo");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        leftOdometry = hardwareMap.get(DcMotor.class, "leftOdoEncoder");
        middleOdometry = hardwareMap.get(DcMotor.class, "middleOdoEncoder");
        rightOdometry = hardwareMap.get(DcMotor.class, "rightOdoEncoder");
        revBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "revBlinkin");
        leftLineFollower = hardwareMap.get(ColorSensor.class, "leftLineFollower");
        rightLineFollower = hardwareMap.get(ColorSensor.class, "rightLineFollower");
        lineBreakSensor = hardwareMap.get(DistanceSensor.class, "lineBreakSensor");



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Reset odometry encoders before drive motors because some of them
        // share motor ports.
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //direction
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        elevatorHeightControlMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorHeightControlMotor.setTargetPosition(0);
        elevatorHeightControlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeControlServo.setPosition(0.5); //stopped position of intake servo

        RevBlinkinLedDriver.BlinkinPattern initialPattern;
        initialPattern = RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;
        revBlinkin.setPattern(initialPattern);


        //Wait for driver to press play
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //runtime until driver hits stop
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double joystickMovementY = inputScaling(-gamepad1.left_stick_y) * JOYSTICK_MOVEMENT_SENSITIVITY;  // Note: pushing stick forward gives negative value
            double joystickMovementX = inputScaling(gamepad1.left_stick_x) * JOYSTICK_MOVEMENT_SENSITIVITY;
            double yaw = (inputScaling(gamepad1.right_stick_x) * JOYSTICK_ROTATION_SENSITIVITY) * 0.75;
            telemetry.addData("Joystick X/Y: ", "%.02f, %.02f", gamepad1.left_stick_x, gamepad1.left_stick_y);

            // Transform requested movement vector from field-space to robot-space
            double robotHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            telemetry.addData("Robot Heading: ", robotHeading);
            telemetry.addData("Input Movement: ", "%.02f, %.02f", joystickMovementX, joystickMovementY);
            double theta = -robotHeading;
            double movementX = joystickMovementX * cos(toRadians(theta)) - joystickMovementY * sin(toRadians(theta));
            double movementY = joystickMovementX * sin(toRadians(theta)) + joystickMovementY * cos(toRadians(theta));
            telemetry.addData("Output Movement: ", "%.02f, %.02f", movementX, movementY);


            if (gamepad1.left_trigger > 0.000) {
                movementX = movementX * 0.45;
                movementY = movementY * 0.45;
                yaw = yaw * 0.45;
            }
            if (gamepad1.left_trigger > 0.000 && gamepad1.left_trigger < 0.001) {
                movementX = movementX / 0.45;
                movementY = movementY / 0.45;
                yaw = yaw / 0.45;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = (movementY + movementX + yaw) * 0.80;
            double rightFrontPower = (movementY - movementX - yaw) * 0.80;
            double leftBackPower = (movementY - movementX + yaw) * 0.80;
            double rightBackPower = (movementY + movementX - yaw) * 0.80;




            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            rightFrontDriveMotor.setPower(rightFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);
            leftFrontDriveMotor.setPower(leftFrontPower);
            leftBackDriveMotor.setPower(leftBackPower);

            // elevator manual control
            double elevatorDelta = (-gamepad2.right_stick_y * ELEVATOR_ADJUST_RATE) * 1.5;
            targetElevatorPosition += elevatorDelta;
            if (gamepad2.a) {
                targetElevatorPosition = ELEVATOR_HEIGHT_LOW;
            }
            if (gamepad2.b) {
                targetElevatorPosition = ELEVATOR_HEIGHT_MIDDLE;
            }
            if (gamepad2.y) {
                targetElevatorPosition = ELEVATOR_HEIGHT_HIGH;
            }
            if (gamepad2.x) {
                targetElevatorPosition = ELEVATOR_HEIGHT_SHORT;
            }
            if (gamepad2.right_bumper) {
                targetElevatorPosition = ELEVATOR_HEIGHT_BOTTOM;
            }

            targetElevatorPosition = max(0.0, targetElevatorPosition); // Cannot go below 0
            targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX); // Cannot go beyond max height


            // Servo intake and limit switch functionality
            double SERVO_UP = 1.0;
            double SERVO_DOWN = 0.0;
            double SERVO_STOP = 0.5;


            if (limitSwitch.getState() == false) {
                SERVO_UP = 0.5;
            }
            if (limitSwitch.getState() == true) {
                SERVO_UP = 1.0;
            }

            if (gamepad2.left_stick_y > 0) {
                intakeControlServo.setPosition(SERVO_DOWN);
            }
            if (gamepad2.left_stick_y == 0) {
                intakeControlServo.setPosition(SERVO_STOP);
            }
            if (gamepad2.left_stick_y < 0) {
                intakeControlServo.setPosition(SERVO_UP);
            }
            if (gamepad1.dpad_down) {
                intakeControlServo.setPosition(SERVO_DOWN);
            }
            if (gamepad1.dpad_up) {
                intakeControlServo.setPosition(SERVO_UP);
            }

            //elevator down and servo intake while trigger is held
            double elevatorTrigger = (gamepad2.left_trigger * ELEVATOR_ADJUST_RATE) * 1.75;
            double testValue = gamepad2.left_trigger;
            targetElevatorPosition -= elevatorTrigger;

            if (gamepad2.left_trigger > 0) {
                intakeControlServo.setPosition(SERVO_UP);
                if (limitSwitch.getState() == false) {
                    targetElevatorPosition = ELEVATOR_HEIGHT_LOW;
                }
            }

            if (gamepad2.left_trigger > 0.0000 && gamepad2.left_trigger < 0.0001) {
                intakeControlServo.setPosition(SERVO_STOP);
                if (limitSwitch.getState() == false) {
                    targetElevatorPosition = ELEVATOR_HEIGHT_LOW;
                }
            }

            //LED Control when switch is actuated
            if (limitSwitch.getState() == false) {
                RevBlinkinLedDriver.BlinkinPattern switchPattern;
                switchPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                revBlinkin.setPattern(switchPattern);
            } else {
                revBlinkin.setPattern(initialPattern);
            }

            //Quick adjust elevator using bumper


            if (targetElevatorPosition == ELEVATOR_HEIGHT_HIGH){
                if (gamepad2.left_bumper == true){
                    targetElevatorPosition = ELEVATOR_QUICK_ADJUST_HIGH_DOWN;
                }
            }
            if (targetElevatorPosition == ELEVATOR_HEIGHT_MIDDLE){
                if (gamepad2.left_bumper == true){
                    targetElevatorPosition = ELEVATOR_QUICK_ADJUST_MIDDLE;
                }
            }
            if (targetElevatorPosition == ELEVATOR_HEIGHT_LOW){
                if (gamepad2.left_bumper == true){
                    targetElevatorPosition = ELEVATOR_QUICK_ADJUST_LOW;
                }
            }

            if (gamepad1.y){
                imu.initialize(parameters);
            }


            //elevator values and switch state
            telemetry.addData("Switch State: ", limitSwitch.getState());
            telemetry.addData("Elevator Position", elevatorHeightControlMotor.getCurrentPosition());
            telemetry.addData("Elevator Target", targetElevatorPosition);
            telemetry.addData("Elevator Delta", elevatorDelta);
            elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
            elevatorHeightControlMotor.setPower(1.0);
            //motor power outputs
            telemetry.addData("Front Left Power: ", leftFrontDriveMotor.getPower());
            telemetry.addData("Front Right Power: ", rightFrontDriveMotor.getPower());
            telemetry.addData("Back Left Power: ", leftBackDriveMotor.getPower());
            telemetry.addData("Back Right Power: ", rightBackDriveMotor.getPower());
            //odometry outputs
            telemetry.addData("Left Odometry Value: ", leftOdometry.getCurrentPosition());
            telemetry.addData("Middle Odometry Value: ", middleOdometry.getCurrentPosition());
            telemetry.addData("Right Odometry Value: ", rightOdometry.getCurrentPosition());

            telemetry.addData("Left Trigger Output: ", gamepad1.left_trigger);

            //telemetry.addData("Bumper Count: ", bumperCount);

            telemetry.addData("Status", "Run Time: " + runtime);


            telemetry.addData("Left Line Follower: ", leftLineFollower.red());

            telemetry.addData("Right Line Follower: ", rightLineFollower.red());


            telemetry.addData("Line Break Sensor: ", lineBreakSensor.getDistance(DistanceUnit.CM));





            telemetry.update();
        }
    }



    double inputScaling(double x) {
        double sign = Math.signum(x);
        double magnitude = Math.abs(x);
        if (magnitude < JOYSTICK_DEAD_ZONE) {
            magnitude = 0.0;
        } else {
            magnitude = (magnitude - JOYSTICK_DEAD_ZONE) / (1.0 - JOYSTICK_DEAD_ZONE);
        }
        magnitude = Math.pow(magnitude, 2.0);
        return sign * magnitude;
    }


}