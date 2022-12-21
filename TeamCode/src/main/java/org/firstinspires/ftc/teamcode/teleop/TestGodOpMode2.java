package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

@TeleOp(group = "advanced")
public class TestGodOpMode2 extends LinearOpMode {

    DcMotor leftBack;
    DcMotor leftF;
    DcMotor rightBack;
    DcMotor rightF;
    Servo claw;
    DcMotor slide;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    Pose2d targetAVector = new Pose2d(-16, 34, Math.toRadians(0));

    double targetAngle = Math.toRadians(0);

    Pose2d startPose = new Pose2d(-16,60,Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(startPose);

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // left front motor
        leftF = hardwareMap.get(DcMotor.class, "leftFront");
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF.setDirection(DcMotor.Direction.FORWARD);

        // right back motor
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // right front motor
        rightF = hardwareMap.get(DcMotor.class, "rightFront");
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setDirection(DcMotor.Direction.REVERSE);

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int slideInitial = slide.getCurrentPosition();
        slide.setTargetPosition(slideInitial);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        claw = hardwareMap.get(Servo.class, "claw");

        int SL_LOW = 1025;
        int SL_MEDIUM = 1575;
        int SL_HIGH = 2175;

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDU2 = true, releasedDD2 = true;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio, but only when
                    // at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    leftF.setPower(frontLeftPower * 0.6);
                    leftBack.setPower(backLeftPower * 0.6);
                    rightF.setPower(frontRightPower * 0.6);
                    rightBack.setPower(backRightPower * 0.6);

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToSplineHeading(targetAVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }

                    if (gamepad2.a) {
                        if (releasedA2) {
                            slide.setTargetPosition(slideInitial);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(0.8);
                            releasedA2 = false;
                        }
                    } else if (!releasedA2) {
                        releasedA2 = true;
                    }

                    if (gamepad2.b) {
                        if (releasedB2) {
                            slide.setTargetPosition(slideInitial + SL_LOW);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(-0.8);
                            releasedB2 = false;
                        }
                    } else if (!releasedB2) {
                        releasedB2 = true;
                    }

                    if (gamepad2.x) {
                        if (releasedX2) {
                            slide.setTargetPosition(slideInitial + SL_MEDIUM);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(0.8);
                            releasedX2 = false;
                        }
                    } else if (!releasedX2) {
                        releasedX2 = true;
                    }

                    if (gamepad2.y) {
                        if (releasedY2) {
                            slide.setTargetPosition(slideInitial + SL_HIGH);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(0.8);
                            releasedY2 = false;
                        }
                    } else if (!releasedY2) {
                        releasedY2 = true;
                    }

                    if (gamepad2.dpad_up) {
                        if (releasedDU2) {
                            slide.setTargetPosition(slide.getCurrentPosition() + 150);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(0.8);
                            releasedDU2 = false;
                        }
                    } else if (!releasedDU2) {
                        releasedDU2 = true;
                    }

                    if (gamepad2.dpad_down) {
                        if (releasedDD2) {
                            slide.setTargetPosition(slide.getCurrentPosition() - 105);
                            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide.setPower(0.8);
                            releasedDD2 = false;
                        }
                    } else if (!releasedDD2) {
                        releasedDD2 = true;
                    }

                    if ((gamepad2.right_bumper)) {
                        claw.setPosition(0.8);
                    } else {
                        claw.setPosition(0.5);
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}

