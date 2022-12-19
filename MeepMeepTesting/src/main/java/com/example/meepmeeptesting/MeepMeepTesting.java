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
                .setConstraints(38, 30, Math.toRadians(140), Math.toRadians(140), 14.01)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 61.2, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-34,12.2, Math.toRadians(270)))
                                .strafeLeft(10.7)
                                .forward(1)
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.2)
                                .back(3)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 400, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-53,11.7, Math.toRadians(180)))
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.2)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .7);
                                })
                                .back(13)
                                .lineToSplineHeading(new Pose2d(-23,12.2, Math.toRadians(270)))
                                .forward(1)
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.2)
                                .back(3)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}