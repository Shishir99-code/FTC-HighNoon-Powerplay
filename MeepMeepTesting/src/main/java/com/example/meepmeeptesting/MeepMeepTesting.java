package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38, 30, Math.toRadians(140), Math.toRadians(140), 14.01)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 60, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .9);
                                })
                                .splineTo(new Vector2d(-34,10.2), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-21.5,10.2), Math.toRadians(270))
                                //.addTemporalMarker(() -> claw.setPosition(0.5))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 350, .9);//power was 0.7
                                })
                                .waitSeconds(.1)
                                .splineToSplineHeading(new Pose2d(-54,10.2, Math.toRadians(180)), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}