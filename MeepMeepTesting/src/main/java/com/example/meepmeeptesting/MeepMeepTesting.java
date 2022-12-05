package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42, 32, Math.toRadians(154), Math.toRadians(154), 14.01)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 58.5, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                   //slideTo(slideInitial + SL_HIGH, 0.7);
                                })
                                .forward(46)//46
                                .turn(Math.toRadians(-45))
                                .forward(0.5)
                                .waitSeconds(1)
                                .addTemporalMarker(4.37, () ->{
                                    //claw.setPosition(0.5)
                                })


                                .back(0.5)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 600, 0.7);
                                })
                                .turn(Math.toRadians(135))
                                .forward(21)
                                .waitSeconds(1)
                                .addTemporalMarker(8.83, () -> {
                                    //claw.setPosition(1)
                                })


                                .back(21)
                                .turn(Math.toRadians(45))
                                .forward(0.5)
                                .waitSeconds(1)
                                .addTemporalMarker(4.37, () ->{
                                    //claw.setPosition(0.5)
                                })


                                .back(0.5)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 600, 0.7);
                                })
                                .turn(Math.toRadians(-45))
                                .forward(21)
                                .waitSeconds(1)
                                .addTemporalMarker(8.83, () -> {
                                    //claw.setPosition(1)
                                })


                                .back(21)
                                .turn(Math.toRadians(135))
                                .forward(0.5)
                                .waitSeconds(1)
                                .addTemporalMarker(4.37, () ->{
                                    //claw.setPosition(0.5)
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}