package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static java.lang.Math.max;
import static java.lang.Math.min;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Score And Park Left")
public class ScoreAndParkLeft extends LinearOpMode {

    private SignalDetectTest sleeveDetection;
    private OpenCvCamera camera;

    private DcMotor elevatorHeightControlMotor = null;
    private Servo intakeControlServo = null;
    private DigitalChannel limitSwitch = null;

    private double targetElevatorPosition = 0;
    //TODO: increase elevator height values for new pulley system
    final double ELEVATOR_HEIGHT_MAX = 4157;
    final double ELEVATOR_HEIGHT_LOW = 1673;
    final double ELEVATOR_HEIGHT_MIDDLE = 2784;
    final double ELEVATOR_HEIGHT_HIGH = 3920;
    final double ELEVATOR_HEIGHT_SHORT = 986;
    final double ELEVATOR_HEIGHT_BOTTOM = 301;
    final double FIVE_STACK_HEIGHT = 664;
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

        /*leftLineFollower = hardwareMap.get(ColorSensor.class, "leftLineFollower");
        rightLineFollower = hardwareMap.get(ColorSensor.class, "rightLineFollower");
        lineBreakSensor = hardwareMap.get(DistanceSensor.class, "lineBreakSensor");*/

        waitForStart();
        camera.closeCameraDevice();

        if (isStopRequested()) return;

        double SERVO_UP = 1.0;
        double SERVO_DOWN = 0.0;
        double SERVO_STOP = 0.5;



        sleep(1000);

        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        /*float hsvValues[] = {0F,0F,0F};

        final float values[] = hsvValues;

        Color.RGBToHSV(leftLineFollower.red() * 8, leftLineFollower.green() * 8, leftLineFollower.blue() * 8, hsvValues);

        Color.RGBToHSV(rightLineFollower.red() * 8, rightLineFollower.green() * 8, rightLineFollower.blue() * 8, hsvValues);*/

        /*TrajectorySequence loadForward = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-35, -54),
                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();*/



        TrajectorySequence initialForward = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-35, -35))
                .build();

        TrajectorySequence lineUpWithJunction = drive.trajectorySequenceBuilder(initialForward.end())
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(-22.6, -35))
                .build();


        TrajectorySequence junctionForward = drive.trajectorySequenceBuilder(lineUpWithJunction.end())
                .lineTo(new Vector2d(-22.6, -30.5))
                .build();

        TrajectorySequence leaveJunction = drive.trajectorySequenceBuilder(junctionForward.end())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-22.5, -35))
                .build();

        TrajectorySequence connectingRegion = drive.trajectorySequenceBuilder(leaveJunction.end())
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-35, -35.1))
                .build();

        TrajectorySequence pushSignal = drive. trajectorySequenceBuilder(connectingRegion.end())
                .lineTo(new Vector2d(-35, -7))
                .build();

        TrajectorySequence alignWithStack = drive.trajectorySequenceBuilder(pushSignal.end())
                .waitSeconds(0.1)
                .lineTo(new Vector2d(-35, -10.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-49, -10.6))
                .build();

        TrajectorySequence stackForward = drive.trajectorySequenceBuilder(alignWithStack.end())
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-61.85, -10.6))
                .build();

        TrajectorySequence leaveStackOrJunction = drive.trajectorySequenceBuilder(stackForward.end())
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-35, -11.5))
                .build();

        TrajectorySequence angleJunctionForward = drive.trajectorySequenceBuilder(leaveStackOrJunction.end())
                .turn(Math.toRadians(140))
                .lineTo(new Vector2d(-30, -17))
                .build();



      /*  TrajectorySequence realignWithStack = drive.trajectorySequenceBuilder(leaveStackOrJunction.end())
                .waitSeconds(0.3)
                .turn(Math.toRadians(-135))
                .build();*/

        TrajectorySequence leaveAngleJunction = drive.trajectorySequenceBuilder(angleJunctionForward.end())
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.4))
                .build();


        TrajectorySequence endAngle = drive.trajectorySequenceBuilder(leaveAngleJunction.end())
                .waitSeconds(0.3)
                .turn(Math.toRadians(-230))
                .build();

        TrajectorySequence colorAdjustLeft = drive.trajectorySequenceBuilder(alignWithStack.end())
                .strafeLeft(0.5)
                .turn(Math.toRadians(1.1))
                .build();

        TrajectorySequence colorAdjustRight = drive.trajectorySequenceBuilder(alignWithStack.end())
                .strafeRight(0.5)
                .turn(Math.toRadians(-1.1))
                .build();



        TrajectorySequence parkingZoneTwo = drive.trajectorySequenceBuilder(endAngle.end())
                .lineTo(new Vector2d(-35, -35.01))
                .build();

        TrajectorySequence parkingZoneOne = drive.trajectorySequenceBuilder(parkingZoneTwo.end())
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-60, -35))
                .build();

        TrajectorySequence parkingZoneThree = drive.trajectorySequenceBuilder(parkingZoneTwo.end())
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-11, -35))
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


        if (sleeveDetection.getPosition() == SignalDetectTest.ParkingPosition.ONE){
            drive.followTrajectorySequence(parkingZoneOne);

        }

        if (sleeveDetection.getPosition() == SignalDetectTest.ParkingPosition.THREE){
            drive.followTrajectorySequence(parkingZoneThree);

        }


        /*
        drive.followTrajectorySequence(pushSignal);

        drive.followTrajectorySequence(alignWithStack);


        targetElevatorPosition = ELEVATOR_HEIGHT_LOW;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);
        targetElevatorPosition = max(0.0, targetElevatorPosition);
        targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
        intakeControlServo.setPosition(SERVO_UP);

        drive.followTrajectorySequence(stackForward);



        double intake2 = 0;
        while (intake2 < 1){
            if (limitSwitch.getState() == true){
                while (limitSwitch.getState() == true){
                    targetElevatorPosition = targetElevatorPosition - 15;
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
                if (targetElevatorPosition >= 2780 && targetElevatorPosition <= 2800){
                    intake2 = intake2 + 1;
                }
            }

        }

        drive.followTrajectorySequence(leaveStackOrJunction);

        drive.followTrajectorySequence(angleJunctionForward);

        intakeControlServo.setPosition(SERVO_DOWN);

        drive.followTrajectorySequence(leaveAngleJunction);

        intakeControlServo.setPosition(SERVO_STOP);
        targetElevatorPosition = ELEVATOR_HEIGHT_SHORT;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);

        drive.followTrajectorySequence(endAngle);
         */
    }
}