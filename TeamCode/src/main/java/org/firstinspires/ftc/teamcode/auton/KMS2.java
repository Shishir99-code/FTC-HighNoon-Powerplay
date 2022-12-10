/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous
public class KMS2 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()

    static final double FEET_PER_METER = 3.28084;
    DcMotor slide;
    Servo claw;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        slide  = hardwareMap.get(DcMotor.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");


        int slideInitial = slide.getCurrentPosition();
        slide.setTargetPosition(slideInitial);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int SL_LOW = 600;
        int SL_MEDIUM = 1450;
        int SL_HIGH = 2280;

        double strafeAdditive = 12;

        claw = hardwareMap.get(Servo.class, "claw");

        Pose2d startPose = new Pose2d(-36,61.2,Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence RightTrajectory = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial + SL_HIGH, .7);
                })
                .forward(49)
                .strafeLeft(16)
                .forward(2.5)
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(0.2);
                })
                .waitSeconds(.1)
                .back(2.5)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial + 400, .7);
                })
                .lineToSplineHeading(new Pose2d(-54, 12.2, Math.toRadians(180)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(1);
                })
                .waitSeconds(0.1)
                .back(15)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial+ SL_HIGH, .7);
                })
                .lineToSplineHeading(new Pose2d(-21, 12.2, Math.toRadians(270)))
                .forward(2.5)
                .waitSeconds(.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(0.2);
                })
                .waitSeconds(.1)
                .back(2.5)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial + 300, .7);
                })
                .lineToSplineHeading(new Pose2d(-54, 12.2, Math.toRadians(180)))
                .waitSeconds(.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(0.8);
                })
                .waitSeconds(0.1)
                .back(15)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial+ SL_HIGH, .7);
                })
                .lineToSplineHeading(new Pose2d(-21, 12.2, Math.toRadians(270)))
                .forward(2.5)
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(0.2);
                })
                .waitSeconds(.100)
                .back(2.5)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial + 200, .7);
                })
                .lineToSplineHeading(new Pose2d(-54, 12.2, Math.toRadians(180)))
                .waitSeconds(.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(0.8);
                })
                .waitSeconds(0.1)
                .back(15)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial+ SL_HIGH, .7);
                })
                .lineToSplineHeading(new Pose2d(-21, 12.2, Math.toRadians(270)))
                .forward(2.5)
                .waitSeconds(.4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(0.2);
                })
                .waitSeconds(.1)
                .back(2.5)
                .addDisplacementMarker(() -> {
                    slideTo(slideInitial + 4, .1);
                })
                .strafeRight(12)
                .build();




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {


            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        System.out.println("BEFIRE!!!");
        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){



        }else if(tagOfInterest.id == MIDDLE){

            claw.setPosition(0.8);
            sleep(1000);
            drive.followTrajectorySequence(RightTrajectory);

        }else if (tagOfInterest.id == RIGHT){


        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void slideTo(int targetPosition, double power) {
        slide.setTargetPosition(targetPosition);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
    }
}