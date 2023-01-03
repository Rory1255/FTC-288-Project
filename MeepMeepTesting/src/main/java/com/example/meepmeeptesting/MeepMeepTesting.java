package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 11.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                                .lineTo(new Vector2d(-35, -54))
                                .waitSeconds(0.3)
                                .lineTo(new Vector2d(-35, -40))
                                .waitSeconds(0.1)
                                .splineToConstantHeading(new Vector2d(-23.5, -35), Math.toRadians(0))
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
                                .turn(Math.toRadians(-225))
                                .lineTo(new Vector2d(-35, -35))
                                .lineTo(new Vector2d(-12, -35))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/* old cycle trajectory
.lineTo(new Vector2d(-35, -11.5))
                                .lineTo(new Vector2d(-57, -11.5))
                                .waitSeconds(0.70)
                                .lineTo(new Vector2d(15, -11.5))
                                .lineTo(new Vector2d(15, -23.5))
                                .lineTo(new Vector2d(10, -23.5))
                                .waitSeconds(0.70)
                                .lineTo(new Vector2d(15, -23.5))
                                .lineTo(new Vector2d(15, -11.5))
                                .lineTo(new Vector2d(-57, -11.5))
                                .waitSeconds(0.70)
                                .lineTo(new Vector2d(15, -11.5))
                                .lineTo(new Vector2d(15, -23.5))
                                .lineTo(new Vector2d(10, -23.5))
                                .waitSeconds(0.70)
                                .lineTo(new Vector2d(15, -23.5))
                                .lineTo(new Vector2d(15, -11.5))
                                .lineTo(new Vector2d(-57, -11.5))
                                .waitSeconds(0.70)
                                .lineTo(new Vector2d(15, -11.5))
                                .lineTo(new Vector2d(15, -23.5))
                                .lineTo(new Vector2d(10, -23.5))
                                .waitSeconds(0.70)
                                .lineTo(new Vector2d(15, -23.5))

                                .build()
 */