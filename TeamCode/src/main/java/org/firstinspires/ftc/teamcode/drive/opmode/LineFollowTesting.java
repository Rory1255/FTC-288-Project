package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "LineFollow")
public class LineFollowTesting extends LinearOpMode {
    private ColorSensor leftLineFollower = null;
    private ColorSensor rightLineFollower = null;
    private BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        leftLineFollower = hardwareMap.get(ColorSensor.class, "leftLineFollower");
        rightLineFollower = hardwareMap.get(ColorSensor.class, "rightLineFollower");

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;
        sleep(1000);

        Pose2d startPose = new Pose2d(49, -9.25, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence adjustLeft1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(1)
                .build();

        TrajectorySequence adjustRight1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(1)
                .build();

        TrajectorySequence adjustLeft2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.2)
                .strafeLeft(0.75)
                .build();

        TrajectorySequence adjustRight2 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.2)
                .strafeRight(0.75)
                .build();

        TrajectorySequence adjustLeft3 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(1)
                .build();

        TrajectorySequence adjustRight3 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(1)
                .build();

        TrajectorySequence adjustLeft4 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(0.1)
                .build();

        TrajectorySequence adjustRight4 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(0.1)
                .build();

        TrajectorySequence stackForward = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.2)
                .forward(10)
                .build();

        TrajectorySequence abort = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .waitSeconds(0.5)
                .turn(Math.toRadians(90))
                .build();

        while (!isStopRequested()) {
            // In an actual autonomous program this function should be called once
            // at the appropriate point in time between trajectories. It will look
            // at color sensor values and raw IMU readings in order to line up the
            // robot.
            alignToMarker(drive, true, -90.0);
        }
    }

    void alignToMarker(MecanumDrive drive, boolean blueMode, double targetAngle) {
        // We can take at most 1s to align the robot.
        double deadline = getRuntime() + 1.0;
        while (!isStopRequested() && getRuntime() < deadline) {
            if (leftLineFollower.red() > leftLineFollower.blue() || rightLineFollower.red() > rightLineFollower.blue()){
                blueMode = false;
            }
            drive.updatePoseEstimate();
            double strengthL = blueMode ? leftLineFollower.blue() : leftLineFollower.red();
            double strengthR = blueMode ? rightLineFollower.blue() : rightLineFollower.red();
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
