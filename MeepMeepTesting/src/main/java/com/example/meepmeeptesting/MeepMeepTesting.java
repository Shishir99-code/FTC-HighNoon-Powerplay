package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45.9, 35.9, Math.toRadians(140), Math.toRadians(140), 14.01)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 61.2, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .7);
                                })
                                .forward(51.5)
                                .strafeLeft(13)
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.2);
                                })
                                .waitSeconds(.25)
                                .back(3.5)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 400, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-54, 12.2, Math.toRadians(180)))
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.8);
                                })
                                .waitSeconds(0.1)
                                .back(18)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial+ SL_HIGH, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-31, 7.2, Math.toRadians(315)))
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(-35, 12.2, Math.toRadians(180)))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 250, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-54, 12.2, Math.toRadians(180)))
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.8);
                                })
                                .waitSeconds(0.1)
                                .back(15)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial+ SL_HIGH, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-31, 7.2, Math.toRadians(315)))
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(-35, 12.2, Math.toRadians(180)))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 250, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-54, 12.2, Math.toRadians(180)))
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.8);
                                })
                                .waitSeconds(0.1)
                                .back(15)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial+ SL_HIGH, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-31, 7.2, Math.toRadians(315)))
                                .waitSeconds(0.4)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.25)
                                .lineToSplineHeading(new Pose2d(-35, 12.2, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}