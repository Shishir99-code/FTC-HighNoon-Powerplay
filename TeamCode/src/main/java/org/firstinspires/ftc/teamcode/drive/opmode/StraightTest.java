package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    DcMotor slide;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        slide  = hardwareMap.get(DcMotor.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int slideInitial = slide.getCurrentPosition();
        slide.setTargetPosition(slideInitial);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = hardwareMap.get(Servo.class, "claw");

        int SL_LOW = 1600;
        int SL_MEDIUM = 2450;
        int SL_HIGH = 3300;

        Pose2d startPose = new Pose2d(30,63.5,Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajblueleft = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() ->  {
                    slideTo(slideInitial + SL_HIGH, 0.6);
                })
                .lineToSplineHeading(new Pose2d(38, 55.5, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(34, 2, Math.toRadians(180)))
                .forward(2)
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> claw.setPosition(0.5))
                .build();

        TrajectorySequence trajToStack = drive.trajectorySequenceBuilder(trajblueleft.end())
                .addTemporalMarker(1, () -> {
                    slideTo(slideInitial + SL_LOW, 0.8);
                })
                .lineToSplineHeading(new Pose2d(37, 11.5, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(58.5, 11.5, Math.toRadians(0)))
                .build();


        TrajectorySequence trajToJunction = drive.trajectorySequenceBuilder(trajToStack.end())
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial + SL_HIGH, 0.7);
                })
                .lineToSplineHeading(new Pose2d(24, 14, Math.toRadians(270)))
                .forward(3)
                .build();

        waitForStart();

        if (isStopRequested()) return;


        claw.setPosition(1);
        sleep(1000);
        drive.followTrajectorySequence(trajblueleft);
        drive.followTrajectorySequence(trajToStack);
        claw.setPosition(1);
        sleep(700);
        slideTo(slide.getCurrentPosition() + 550, 1);
        sleep(600);
        drive.followTrajectorySequence(trajToJunction);
        claw.setPosition(0.5);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    private void slideTo(int targetPosition, double power) {
        slide.setTargetPosition(targetPosition);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
    }
}