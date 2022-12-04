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

        int SL_LOW = 1600;
        int SL_MEDIUM = 2450;
        int SL_HIGH = 3300;

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDU2 = true, releasedDD2 = true;


        waitForStart();
        while (opModeIsActive()) {

            // Initiallizing vairables
            double horizontal = 0.6 * gamepad1.left_stick_x;
            double vertical = -0.6 * gamepad1.left_stick_y;
            double turn = 0.6 * gamepad1.right_stick_x;


            // equations for movement
            leftFront.setPower(vertical + horizontal + turn);
            leftBack.setPower(vertical - horizontal + turn);
            rightFront.setPower(vertical - horizontal - turn);
            rightBack.setPower(vertical + horizontal - turn);

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
                claw.setPosition(1);
            } else {
                claw.setPosition(0.5);
            }


        }

    }
}