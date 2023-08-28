package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous(name = "Park Only")
public class ParkAndLoadOnly extends LinearOpMode {

    private SignalDetectTest sleeveDetection;
    private OpenCvCamera camera;

    private DcMotor elevatorHeightControlMotor = null;
    private Servo intakeControlServo = null;
    private DigitalChannel limitSwitch = null;

    private double targetElevatorPosition = 0;
    //TODO: increase elevator height values for new pulley system
    final double ELEVATOR_HEIGHT_MAX = 4157;
    final double ELEVATOR_HEIGHT_LOW = 1773;
    final double ELEVATOR_HEIGHT_MIDDLE = 2905;
    final double ELEVATOR_HEIGHT_HIGH = 3960;
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

        waitForStart();
        camera.closeCameraDevice();

        if (isStopRequested()) return;

        double SERVO_UP = 1.0;
        double SERVO_DOWN = 0.0;
        double SERVO_STOP = 0.5;



        sleep(1000);

        Pose2d startPose = new Pose2d(35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);



        TrajectorySequence initialForward = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .lineTo(new Vector2d(35, -35))
                .build();

        Trajectory parkingZoneOne = drive.trajectoryBuilder(initialForward.end())
                .lineTo(new Vector2d(60, -35))
                .build();

       /* Trajectory parkingZoneTwo = drive.trajectoryBuilder(initialForward.end())
                .lineTo(new Vector2d(35, -35))
                .build();*/

        Trajectory parkingZoneThree = drive.trajectoryBuilder(initialForward.end())
                .lineTo(new Vector2d(12, -35))
                .build();


        drive.followTrajectorySequence(initialForward);


        if (sleeveDetection.getPosition() == SignalDetectTest.ParkingPosition.ONE){
            drive.followTrajectory(parkingZoneOne);

        }

        if (sleeveDetection.getPosition() == SignalDetectTest.ParkingPosition.THREE){
            drive.followTrajectory(parkingZoneThree);

        }
    }
}