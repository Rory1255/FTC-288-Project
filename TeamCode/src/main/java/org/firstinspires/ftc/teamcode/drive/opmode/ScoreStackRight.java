package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Score From Stack Right")
public class ScoreStackRight extends LinearOpMode {

    private SignalDetectTest sleeveDetection;
    private OpenCvCamera camera;

    private DcMotor elevatorHeightControlMotor = null;
    private Servo intakeControlServo = null;
    private DigitalChannel limitSwitch = null;

    private RevBlinkinLedDriver revBlinkin = null;

    private ColorSensor leftLineFollower = null;
    private ColorSensor rightLineFollower = null;

    private BNO055IMU imu;

    private double targetElevatorPosition = 0;
    //TODO: increase elevator height values for new pulley system
    final double ELEVATOR_HEIGHT_MAX = 4157;
    final double ELEVATOR_HEIGHT_LOW = 1673;
    final double ELEVATOR_HEIGHT_MIDDLE = 2784;
    final double ELEVATOR_HEIGHT_HIGH = 3920;
    final double ELEVATOR_HEIGHT_SHORT = 986;
    final double ELEVATOR_HEIGHT_BOTTOM = 301;
    final double FIVE_STACK_HEIGHT = 1174;
    final double FOUR_STACK_HEIGHT = 494;
    final double THREE_STACK_HEIGHT = 327;
    final double TWO_STACK_HEIGHT = 161;
    final double GROUND_CONE_HEIGHT = 13;
    final double ELEVATOR_ADJUST_RATE = 15.0;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SignalDetectTest();
        camera.setPipeline(sleeveDetection);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevatorHeightControlMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorHeightControlMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorHeightControlMotor.setTargetPosition(0);
        elevatorHeightControlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeControlServo = hardwareMap.get(Servo.class, "intakeServo");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        revBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "revBlinkin");

        leftLineFollower = hardwareMap.get(ColorSensor.class, "leftLineFollower");
        rightLineFollower = hardwareMap.get(ColorSensor.class, "rightLineFollower");
        //lineBreakSensor = hardwareMap.get(DistanceSensor.class, "lineBreakSensor");

        waitForStart();
        camera.closeCameraDevice();

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        if (isStopRequested()) return;

        double SERVO_UP = 1.0;
        double SERVO_DOWN = 0.0;
        double SERVO_STOP = 0.5;

        double leftRedValue = leftLineFollower.red();
        double rightRedValue = rightLineFollower.red();
        double leftBlueValue = leftLineFollower.blue();
        double rightBlueValue = rightLineFollower.blue();

        RevBlinkinLedDriver.BlinkinPattern initialPattern;
        initialPattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        revBlinkin.setPattern(initialPattern);

        sleep(1000);

        Pose2d startPose = new Pose2d(35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        TrajectorySequence initialForward = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(35, -35))
                .build();

        TrajectorySequence lineUpWithJunction = drive.trajectorySequenceBuilder(initialForward.end())
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(22.6, -35))
                .build();


        TrajectorySequence junctionForward = drive.trajectorySequenceBuilder(lineUpWithJunction.end())
                .lineTo(new Vector2d(22.6, -30.5),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence leaveJunction = drive.trajectorySequenceBuilder(junctionForward.end())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(22.5, -35))
                .build();

        TrajectorySequence connectingRegion = drive.trajectorySequenceBuilder(leaveJunction.end())
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(35, -35.1))
                .build();

        TrajectorySequence pushSignal = drive. trajectorySequenceBuilder(connectingRegion.end())
                .lineTo(new Vector2d(35, -7))
                .build();

        TrajectorySequence alignWithStack = drive.trajectorySequenceBuilder(pushSignal.end())
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(35, -9.25))
                .turn(Math.toRadians(-92))
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(49, -9.25),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence stackForward = drive.trajectorySequenceBuilder(alignWithStack.end())
                .waitSeconds(0.3)
                .lineTo(new Vector2d(60.2, -9.25),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence leaveStackOrJunction = drive.trajectorySequenceBuilder(stackForward.end())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(35, -9.25))
                .build();

        TrajectorySequence angleJunctionForward = drive.trajectorySequenceBuilder(leaveStackOrJunction.end())
                .turn(Math.toRadians(-150))
                .lineTo(new Vector2d(29.9, -16.9))
                .build();



        TrajectorySequence leaveAngleJunction = drive.trajectorySequenceBuilder(angleJunctionForward.end())
                .waitSeconds(0.3)
                .lineTo(new Vector2d(35, -11.4))
                .build();


        TrajectorySequence endAngle = drive.trajectorySequenceBuilder(leaveAngleJunction.end())
                .waitSeconds(0.3)
                .turn(Math.toRadians(240))
                .build();



        TrajectorySequence parkingZoneTwo = drive.trajectorySequenceBuilder(endAngle.end())
                .waitSeconds(0.6)
                .lineTo(new Vector2d(35, -35.01))
                .build();

        TrajectorySequence parkingZoneOne = drive.trajectorySequenceBuilder(parkingZoneTwo.end())
                .waitSeconds(0.2)
                .lineTo(new Vector2d(11, -35))
                .build();

        TrajectorySequence parkingZoneThree = drive.trajectorySequenceBuilder(parkingZoneTwo.end())
                .waitSeconds(0.2)
                .lineTo(new Vector2d(60, -35))
                .build();


        targetElevatorPosition = ELEVATOR_HEIGHT_MIDDLE;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);
        targetElevatorPosition = max(0.0, targetElevatorPosition);
        targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
        intakeControlServo.setPosition(SERVO_UP);

        drive.followTrajectorySequence(initialForward);


        drive.followTrajectorySequence(lineUpWithJunction);
        drive.followTrajectorySequence(junctionForward);

        intakeControlServo.setPosition(SERVO_DOWN);

        drive.followTrajectorySequence(leaveJunction);

        intakeControlServo.setPosition(SERVO_STOP);
        targetElevatorPosition = ELEVATOR_HEIGHT_BOTTOM;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);


        drive.followTrajectorySequence(connectingRegion);

        drive.followTrajectorySequence(pushSignal);

        drive.followTrajectorySequence(alignWithStack);


        targetElevatorPosition = FIVE_STACK_HEIGHT;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);
        targetElevatorPosition = max(0.0, targetElevatorPosition);
        targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
        intakeControlServo.setPosition(SERVO_UP);

        if (!isStopRequested()) {
            // In an actual autonomous program this function should be called once
            // at the appropriate point in time between trajectories. It will look
            // at color sensor values and raw IMU readings in order to line up the
            // robot.
            alignToMarker(drive, true, -90.0);

            if (leftRedValue > 200 && rightRedValue > 200){
                RevBlinkinLedDriver.BlinkinPattern redDetect;
                redDetect = RevBlinkinLedDriver.BlinkinPattern.RED;
                revBlinkin.setPattern(redDetect);
            }
            if (leftBlueValue > 451 && rightBlueValue > 450){
                RevBlinkinLedDriver.BlinkinPattern blueDetect;
                blueDetect = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                revBlinkin.setPattern(blueDetect);
            }

        }



       // drive.followTrajectorySequence(stackForward);



        double intake2 = 0;
        while (intake2 < 1){
            if (limitSwitch.getState() == true){
                while (limitSwitch.getState() == true){
                    targetElevatorPosition = targetElevatorPosition - 2;
                    elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
                    elevatorHeightControlMotor.setPower(0.8);
                    targetElevatorPosition = max(0.0, targetElevatorPosition);
                    targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
                }
            }
            if (limitSwitch.getState() == false){
                targetElevatorPosition = ELEVATOR_HEIGHT_MIDDLE;
                elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
                elevatorHeightControlMotor.setPower(0.8);
                intakeControlServo.setPosition(SERVO_STOP);
                RevBlinkinLedDriver.BlinkinPattern switchPattern;
                switchPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                revBlinkin.setPattern(switchPattern);
                if (targetElevatorPosition >= 2780 && targetElevatorPosition <= 2800){
                    intake2 = intake2 + 1;
                }
            }

        }



        drive.followTrajectorySequence(leaveStackOrJunction);

        drive.followTrajectorySequence(angleJunctionForward);

        intakeControlServo.setPosition(SERVO_DOWN);

        if (limitSwitch.getState() == true && leftRedValue < 200 && rightRedValue < 200 && leftBlueValue < 450 && rightBlueValue < 450) {
            revBlinkin.setPattern(initialPattern);
        }

        drive.followTrajectorySequence(leaveAngleJunction);

        intakeControlServo.setPosition(SERVO_STOP);
        targetElevatorPosition = ELEVATOR_HEIGHT_SHORT;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);


        drive.followTrajectorySequence(endAngle);
        drive.followTrajectorySequence(parkingZoneTwo);

        if (sleeveDetection.getPosition() == SignalDetectTest.ParkingPosition.ONE){
            drive.followTrajectorySequence(parkingZoneOne);

        }

        if (sleeveDetection.getPosition() == SignalDetectTest.ParkingPosition.THREE){
            drive.followTrajectorySequence(parkingZoneThree);

        }

    }
    void alignToMarker(MecanumDrive drive, boolean blueMode, double targetAngle) {
        // We can take at most 1s to align the robot.
       final double deadline = getRuntime() + 1.0;
        while (!isStopRequested() && getRuntime() < deadline) {

            if (leftLineFollower.red() > leftLineFollower.blue() || rightLineFollower.red() > rightLineFollower.blue()){
                blueMode = false;
            }
            drive.updatePoseEstimate();
            double strengthL = blueMode ? leftLineFollower.blue() : leftLineFollower.red();
            double strengthR = blueMode ? rightLineFollower.blue() : leftLineFollower.red();
            double xdelta = strengthR - strengthL;
            double xpower = xdelta * 0.0002;
            if (strengthL < 300 && strengthR < 300) {
                xpower = 0; // If we're not on the line at all, don't try to align with it
            }

            double angle = imu.getAngularOrientation().firstAngle;
            double angleDelta = angle - targetAngle;
            double rpower = angleDelta * 0.02;
            if (Math.abs(angleDelta) > 15.0) {
                rpower = 0; // If we're way off, don't try to correct angle
            }

            telemetry.addData("Left Reading:", strengthL);
            telemetry.addData("Right Reading:", strengthR);
            telemetry.addData("IMU Angle:", angle);
            telemetry.addData("X Power:", xpower);
            telemetry.addData("R Power:", rpower);
            telemetry.update();

            // TODO: Add a check here such that if the xdelta and angleDelta values
            // are within a reasonable range (say xdelta < 50 and angleDelta < 5 and
            // also strengthL/R are > 300 to make sure we're on the line) for some
            // length of time (this is an important qualifier to make sure we don't
            // overshoot across the line and fail to correct afterwards) we can exit
            // the alignment loop early.

            // Optionally we might drive forward a little bit during this whole alignment process.
            // This helps get us into position *marginally* faster, and also might help by adding
            // some baseline motion so the X/R adjustments are a bit more accurate.
            double ypower = 0.15;

            // Compute motor drive strengths and apply.
            double scale = Math.max(Math.abs(xpower) + Math.abs(ypower) + Math.abs(rpower), 1.0);
            double powerFL = (ypower + xpower + rpower) / scale;
            double powerBL = (ypower - xpower + rpower) / scale;
            double powerFR = (ypower - xpower - rpower) / scale;
            double powerBR = (ypower + xpower - rpower) / scale;
            drive.setMotorPowers(powerFL, powerBL, powerBR, powerFR);
        }
    }

}