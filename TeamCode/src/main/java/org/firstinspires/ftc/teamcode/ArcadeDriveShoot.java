package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 *
 * THIS PROGRAM IS A GOOD TEST OF OUR SYSTEMS.
 * IT WILL DRIVE, RUN THE INTAKE, FIRE A SHOT, AND TURN THE SHOOTER ON AND OFF.
 * IT DOES NOT USE PID OR THE SHOOTER STATE MACHINE.
 *
 * */


@TeleOp(name = "Arcade")
public class ArcadeDriveShoot extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private DcMotor intake;
    private Servo flick;
    private Servo stopper;
    private Servo intakeServo;

    public double flickExtend = 0.7;
    public double flickRetract = 0.48;
    public double stopperClosed = 0.85;
    public double stopperOpen = 1;
    public double intakeServoClosed = 0;
    public double intakeServoOpen = .2;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");
        intakeServo = hardwareMap.servo.get("intakeServo");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        //shoot1.setDirection(DcMotor.Direction.REVERSE);
        //shoot2.setDirection(DcMotor.Direction.REVERSE);

        flick.setPosition(flickRetract);
        stopper.setPosition(stopperClosed);
        intakeServo.setPosition(intakeServoClosed);

        final double PRESS_TIME_MS = 300;
        boolean output = false;
        boolean shooterOn = false;
        double last_rb_press = 0.;
        double last_lb_press = 0.;
        double last_a_press = 0.;
        double last_b_press = 0.;
        double last_back_press = 0.;
        double power = .5;
        int powerPreset = 0;

        waitForStart();

        while (opModeIsActive()) {

            float deadZone = (float) 0.5;
            gamepad1.setJoystickDeadzone(deadZone);

            motorFrontRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);
            motorBackRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorFrontLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorBackLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);

            final double now = System.currentTimeMillis();
            if (gamepad1.right_bumper && (now - last_rb_press > PRESS_TIME_MS)) {
                last_rb_press = now;
                output = !output;
                intake.setPower(output ? 1 : 0);
            }

            if (gamepad1.left_bumper && (now - last_lb_press > PRESS_TIME_MS)) {
                last_lb_press = now;
                output = !output;
                intake.setPower(output ? -1 : 0);
            }

            if (gamepad1.y) {  // SINGLE SHOT
                stopper.setPosition(stopperOpen);
                sleep(200);
                flick.setPosition(flickExtend);
                sleep(100);
                flick.setPosition(flickRetract);
                sleep(200);
                stopper.setPosition(stopperClosed);
            }

            if (gamepad1.x) {  // THREE SHOTS
                stopper.setPosition(stopperOpen);
                sleep(200);
                flick.setPosition(flickExtend);
                sleep(100);
                flick.setPosition(flickRetract);
                sleep(200);
                flick.setPosition(flickExtend);
                sleep(100);
                flick.setPosition(flickRetract);
                sleep(200);
                flick.setPosition(flickExtend);
                sleep(100);
                flick.setPosition(flickRetract);
                sleep(200);
                stopper.setPosition(stopperClosed);
            }

            if (gamepad1.b && (now - last_b_press > PRESS_TIME_MS)) {
                last_b_press = now;
                shooterOn = !shooterOn;
                if (shooterOn) {
                    shoot1.setPower(power);
                    shoot2.setPower(power);
                } else {
                    shoot1.setPower(0);
                    shoot2.setPower(0);
                }
            }

            if (gamepad1.back && (now - last_back_press > PRESS_TIME_MS)) {
                last_back_press = now;
                output = !output;
                intakeServo.setPosition(output ? intakeServoOpen : intakeServoClosed);
            }

            if (gamepad1.start) {
                powerPreset++;
                if (powerPreset == 1) {
                    power = .70;
                    telemetry.addData("preset = WHITE LINE HIGH GOAL : power = ", (Math.round(100 * power)));
                    telemetry.update();
                }
                else if (powerPreset == 2) {
                    power = .80;
                    telemetry.addData("preset = STARTER STACK HIGH GOAL : power = ", (Math.round(100 * power)));
                    telemetry.update();
                }
                else if (powerPreset == 3) {
                    power = .65;
                    powerPreset = 0;
                    telemetry.addData("preset = POWER SHOT : power = ", (Math.round(100 * power)));
                    telemetry.update();
                }
            }

            if (gamepad1.dpad_up) {
                power = power + 0.1;
                telemetry.addData("power = ", (Math.round(100 * power)));
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_down) {
                power = power - 0.1;
                telemetry.addData("power = ", (Math.round(100 * power)));
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_right) {
                power = power + 0.01;
                telemetry.addData("power = ", (Math.round(100 * power)));
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_left) {
                power = power - 0.01;
                telemetry.addData("power = ", (Math.round(100 * power)));
                telemetry.update();
                sleep(500);
            }


        }
        idle();
    }
}



