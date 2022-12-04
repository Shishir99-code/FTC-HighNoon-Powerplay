package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    final double SLIDE_TICKS_PER_REV = 28;
    final double PULLEY_DIAMETER = 1;
    final double GEAR_REDUCTION = 1;
    double COUNTS_PER_INCH_SLIDE = (SLIDE_TICKS_PER_REV * GEAR_REDUCTION) /
            (PULLEY_DIAMETER);

    Servo claw;
    DcMotor slide;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int slideInitial = slide.getCurrentPosition();
        slide.setTargetPosition(slideInitial);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int SL_LOW = 1400;
        int SL_MEDIUM = 2250;
        int SL_HIGH = 3050;

        claw = hardwareMap.get(Servo.class, "claw");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDU2 = true, releasedDD2 = true;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (gamepad2.a) {
                if (releasedA2) {
                    slide.setTargetPosition(slideInitial);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedA2 = false;
                }
            } else if (!releasedA2) {
                releasedA2 = true;
            }

            if (gamepad2.b) {
                if (releasedB2) {
                    slide.setTargetPosition(slideInitial + SL_LOW);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }

            if (gamepad2.x) {
                if (releasedX2) {
                    slide.setTargetPosition(slideInitial + SL_MEDIUM);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }

            if (gamepad2.y) {
                if (releasedY2) {
                    slide.setTargetPosition(slideInitial + SL_HIGH);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }

            if (gamepad2.dpad_up) {
                if (releasedDU2) {
                    slide.setTargetPosition(slide.getCurrentPosition() + 105);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(0.8  );
                    releasedDU2 = false;
                }
            } else if (!releasedDU2){
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

            if ( (gamepad2.dpad_right)) {
                claw.setPosition(0.8);
            }
            else {
                claw.setPosition(0.4);
            }


        }


    }

    public void slideAuton(double inches, double power) {

        int newSlideTarget = 0;

        newSlideTarget = (int)(inches * COUNTS_PER_INCH_SLIDE) - slide.getCurrentPosition();

        slide.setTargetPosition(newSlideTarget);

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setPower(Math.abs(power));

        while (opModeIsActive() && (slide.isBusy())) {
            telemetry.addLine("Slide is Moving");
            telemetry.update();
        }

        slide.setPower(0);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


}