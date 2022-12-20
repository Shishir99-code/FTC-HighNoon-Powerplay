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
                        drive.trajectorySequenceBuilder(new Pose2d(-34, 61.2, Math.toRadians(270)))
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .7);
                                })
                                .lineToSplineHeading(new Pose2d(-34,-2, Math.toRadians(270)))
                                        //SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        //SampleMecanumDrive.getAccelerationConstraint(40))
                                .lineToSplineHeading(new Pose2d(-34,1.2, Math.toRadians(270)))
                                        //SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        //SampleMecanumDrive.getAccelerationConstraint(40))
                                .strafeLeft(18)
                                .waitSeconds(0.1)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);
                                })
                                .waitSeconds(.2)
                                .back(3)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 350, .8);//power was 0.7
                                })
                                .lineToSplineHeading(new Pose2d(-56,3.7, Math.toRadians(180)))
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    //claw.setPosition(1);
                                })
                                .waitSeconds(.2)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .7);
                                })
                                .waitSeconds(.1)
                                .back(13)
                                .lineToSplineHeading(new Pose2d(-16.5,0.2, Math.toRadians(270)))
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);

                                })
                                .waitSeconds(.2)
                                .back(3)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 250, .8);
                                })
                                .lineToSplineHeading(new Pose2d(-56,2.5 , Math.toRadians(180)))
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    // claw.setPosition(1);
                                })
                                .waitSeconds(.3)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + SL_HIGH, .8);
                                })
                                .waitSeconds(.1)
                                .back(13)
                                .lineToSplineHeading(new Pose2d(-16.5,-1.5, Math.toRadians(270)))
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //claw.setPosition(0.5);

                                })
                                .waitSeconds(.1)
                                .back(2)
                                .addDisplacementMarker(() -> {
                                    //slideTo(slideInitial + 4, .8);
                                })
                                .strafeRight(18)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}