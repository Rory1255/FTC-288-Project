package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;

@Disabled
@Config
@Autonomous(name = "RightAuto", group = "drive")
public class RightAutoTestOpMode extends LinearOpMode {
    private DcMotor elevatorHeightControlMotor = null;
    private Servo intakeControlServo = null;
    private DigitalChannel limitSwitch = null;

    private double targetElevatorPosition = 0;

    final double ELEVATOR_HEIGHT_MAX = 4157;
    final double ELEVATOR_HEIGHT_LOW = 1773;
    final double ELEVATOR_HEIGHT_MIDDLE = 2905;
    final double ELEVATOR_HEIGHT_HIGH = 3990;
    final double ELEVATOR_HEIGHT_SHORT = 986;
    final double ELEVATOR_HEIGHT_BOTTOM = 301;
    final double ELEVATOR_ADJUST_RATE = 15.0;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        elevatorHeightControlMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorHeightControlMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorHeightControlMotor.setTargetPosition(0);
        elevatorHeightControlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeControlServo = hardwareMap.get(Servo.class, "intakeServo");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        waitForStart();



        if (isStopRequested()) return;



        double SERVO_UP = 1.0;
        double SERVO_DOWN = 0.0;
        double SERVO_STOP = 0.5;



        sleep(1000);

        Pose2d startingPose = new Pose2d(-35, -62, Math.toRadians(90));

        Trajectory heHeHeHaw = drive.trajectoryBuilder(startingPose)
                .lineTo(new Vector2d(-35, -10))
                .build();






        targetElevatorPosition = ELEVATOR_HEIGHT_MIDDLE;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);

        drive.followTrajectory(heHeHeHaw);




    }
}
