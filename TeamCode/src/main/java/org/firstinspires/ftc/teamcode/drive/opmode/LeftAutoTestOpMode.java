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

//@Disabled
@Config
@Autonomous(name = "LeftAuto", group = "drive")
public class LeftAutoTestOpMode extends LinearOpMode {
    private DcMotor elevatorHeightControlMotor = null;
    private Servo intakeControlServo = null;
    private DigitalChannel limitSwitch = null;

    private double targetElevatorPosition = 0;

    final double ELEVATOR_HEIGHT_MAX = 4157;
    final double ELEVATOR_HEIGHT_LOW = 1773;
    final double ELEVATOR_HEIGHT_MIDDLE = 2905;
    final double ELEVATOR_HEIGHT_HIGH = 4097;
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

        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);




        TrajectorySequence bigTest = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35, -54))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -40))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(-23.5, -35), 0)
                .lineTo(new Vector2d(-23.5, -33))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-23.5, -35))
                .lineTo(new Vector2d(-35, -35))
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-59, -11.5))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(135))
                .lineTo(new Vector2d(-30, -17))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(-135))
                .lineTo(new Vector2d(-59, -11.5))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(135))
                .lineTo(new Vector2d(-30, -17))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(-135))
                .lineTo(new Vector2d(-59, -11.5))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(135))
                .lineTo(new Vector2d(-30, -17))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-35, -11.5))
                .turn(Math.toRadians(135))
                .lineTo(new Vector2d(-35, -35))
                .lineTo(new Vector2d(-12, -35))
                .build();


       /* TrajectorySequence coneLineUp = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-35, -63))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-35, -10))
                .waitSeconds(0.2)
                .build();



        TrajectorySequence conePickupShort = drive.trajectorySequenceBuilder(coneLineUp.end())
                .lineTo(new Vector2d(-61, -10))
                .build();


        TrajectorySequence leaveStack = drive.trajectorySequenceBuilder(conePickupShort.end())
                .waitSeconds(0.7)
                .lineTo(new Vector2d(15, -9.5))
                .build();

        TrajectorySequence alignWithJunction = drive.trajectorySequenceBuilder(leaveStack.end())
                .waitSeconds(0.2)
                .lineTo(new Vector2d(15, -23.5))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(9, -23.5))
                .build();

        TrajectorySequence junctionBackUp = drive.trajectorySequenceBuilder(alignWithJunction.end())
                .waitSeconds(0.7)
                .lineTo(new Vector2d(15, -23))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(15, -9.5))
                .build();

        TrajectorySequence returnToStack = drive.trajectorySequenceBuilder(junctionBackUp.end())
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-57, -10))
                .build();


        drive.followTrajectorySequence(coneLineUp);

        targetElevatorPosition = ELEVATOR_HEIGHT_SHORT;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);
        targetElevatorPosition = max(0.0, targetElevatorPosition);
        targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
        intakeControlServo.setPosition(SERVO_UP);

        drive.followTrajectorySequence(conePickupShort);

        double intake = 0;
        while (intake < 1){
            if (limitSwitch.getState() == true){
                while (limitSwitch.getState() == true){
                    targetElevatorPosition = targetElevatorPosition - 15;
                    elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
                    elevatorHeightControlMotor.setPower(1.0);
                    targetElevatorPosition = max(0.0, targetElevatorPosition);
                    targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
                }
            }
            if (limitSwitch.getState() == false){
                targetElevatorPosition = ELEVATOR_HEIGHT_HIGH;
                elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
                elevatorHeightControlMotor.setPower(1.0);
                intakeControlServo.setPosition(SERVO_STOP);
                if (targetElevatorPosition >= 4080 && targetElevatorPosition <= 4101){
                    intake = intake + 1;
                }
            }

        }

        drive.followTrajectorySequence(leaveStack);

        drive.followTrajectorySequence(alignWithJunction);

        intakeControlServo.setPosition(SERVO_DOWN);

        drive.followTrajectorySequence(junctionBackUp);

        targetElevatorPosition = ELEVATOR_HEIGHT_SHORT;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);
        targetElevatorPosition = max(0.0, targetElevatorPosition);
        targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
        intakeControlServo.setPosition(SERVO_UP);

        drive.followTrajectorySequence(returnToStack);

        drive.followTrajectorySequence(conePickupShort);

        intake = 0;
        while (intake < 1){
            if (limitSwitch.getState() == true){
                while (limitSwitch.getState() == true){
                    targetElevatorPosition = targetElevatorPosition - 15;
                    elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
                    elevatorHeightControlMotor.setPower(1.0);
                    targetElevatorPosition = max(0.0, targetElevatorPosition);
                    targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);
                }
            }
            if (limitSwitch.getState() == false){
                targetElevatorPosition = ELEVATOR_HEIGHT_HIGH;
                elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
                elevatorHeightControlMotor.setPower(1.0);
                intakeControlServo.setPosition(SERVO_STOP);
                if (targetElevatorPosition >= 4080 && targetElevatorPosition <= 4101){
                    intake = intake + 1;
                }
            }

        }

        drive.followTrajectorySequence(leaveStack);

        drive.followTrajectorySequence(alignWithJunction);

        intakeControlServo.setPosition(SERVO_DOWN);

        drive.followTrajectorySequence(junctionBackUp);*/



        targetElevatorPosition = ELEVATOR_HEIGHT_MIDDLE;
        elevatorHeightControlMotor.setTargetPosition((int) targetElevatorPosition);
        elevatorHeightControlMotor.setPower(1.0);
        targetElevatorPosition = max(0.0, targetElevatorPosition);
        targetElevatorPosition = min(targetElevatorPosition, ELEVATOR_HEIGHT_MAX);

        drive.followTrajectorySequence(bigTest);
    }
}
