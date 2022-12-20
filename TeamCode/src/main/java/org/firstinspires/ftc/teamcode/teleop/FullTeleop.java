package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp

public class FullTeleop extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    Servo claw;
    DcMotor slide;


    final double SLIDE_TICKS_PER_REV = 384.5;
    final double PULLEY_DIAMETER = 1;
    final double GEAR_REDUCTION = 1.857;
    double COUNTS_PER_INCH_SLIDE = (SLIDE_TICKS_PER_REV * GEAR_REDUCTION) /
            (PULLEY_DIAMETER);

    @Override
    public void runOpMode() {
        // mapping motors to their correct names
        // left back motor
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // left front motor
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        // right back motor
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // right front motor
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

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
        while (opModeIsActive()) {

            // Initiallizing vairables
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

            leftFront.setPower(frontLeftPower * 0.7);
            leftBack.setPower(backLeftPower * 0.7);
            rightFront.setPower(frontRightPower * 0.7);
            rightBack.setPower(backRightPower * 0.7);

            if ((gamepad1.right_bumper)) {
                rightFront.setPower(1);
                leftBack.setPower(1);
                leftFront.setPower(-1);
                rightBack.setPower(-1);
            }
            if ((gamepad1.left_bumper)) {
                rightFront.setPower(-1);
                leftBack.setPower(-1);
                leftFront.setPower(1);
                rightBack.setPower(1);
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

            telemetry.addData("Counts:", "BL=%d FL=%d BR=%d FR=%d", leftBack.getCurrentPosition(), leftFront.getCurrentPosition(), rightBack.getCurrentPosition(), rightFront.getCurrentPosition());

        }

    }
}