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
                                    //slideTo(slideInitial + SL_HIGH, 7);
                                })
                                .strafeLeft(24)
                                .forward(49)
                                .strafeRight(12)
                                .forward(1)
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .back(1)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 600, 1);
                                })
                                .lineToSplineHeading(new Pose2d(-55, 12.5, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(1);
                                })
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial+ SL_HIGH, 1);
                                })
                                .lineToSplineHeading(new Pose2d(-24, 12.5, Math.toRadians(270)))
                                .forward(1)
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .back(1)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 500, 1);
                                })
                                .lineToSplineHeading(new Pose2d(-55, 12.5, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(1);
                                })
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial+ SL_HIGH, 1);
                                })
                                .lineToSplineHeading(new Pose2d(-24, 12.5, Math.toRadians(270)))
                                .forward(1)
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .back(1)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 350, 1);
                                })
                                .lineToSplineHeading(new Pose2d(-55, 12.5, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(1);
                                })
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial+ SL_HIGH, 1);
                                })
                                .lineToSplineHeading(new Pose2d(-24, 12.5, Math.toRadians(270)))
                                .forward(1)
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .back(1)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 4, 1);
                                })
                                .strafeRight(12)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}