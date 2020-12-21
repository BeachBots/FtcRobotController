package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleAuton;

@TeleOp(name = "FinalTeleOP")
public class finalTeleOp extends LinearOpMode {


    private Servo flick;
    private Servo stopper;
    private DcMotor intake;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor wobbleMotor;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo wobbleClaw;

    private ShooterStateMachine shooter = new ShooterStateMachine();

    private PID pid = new PID();




    public void motortest(double power, int distance) {

        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleMotor.setTargetPosition(-distance);

        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbleMotor.setPower(power);

        while (wobbleMotor.isBusy()) {

        }

        wobbleMotor.setPower(0);

        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void runOpMode() throws InterruptedException {


        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        wobbleMotor = hardwareMap.dcMotor.get("wobbleMotor");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        shooter.init(hardwareMap);
        pid.init(hardwareMap);

        double power = 1.5;

        flick.setPosition(0.48);
        stopper.setPosition(0.85);
        boolean output = false;  // this is for the intake
        boolean shooterOn = false;
        double last_rb_press = 0.;
        double last_lb_press = 0.;
        double last_a_press = 0.;
        final double PRESS_TIME_MS = 300;

        waitForStart();


        while (opModeIsActive()) {

            float deadZone = (float) 0.5;
            gamepad1.setJoystickDeadzone(deadZone);

            motorFrontRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);
            motorBackRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorFrontLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorBackLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);

            // if (gamepad1.square) {
            //    wobbleClaw.setPosition(/*open*/0);
            //     motortest(1, 0/*down*/);
            //     wobbleClaw.setPosition(/*close*/0);
            //     motortest(1, 0/*up*/);
            //  }

            // if (gamepad1.triangle){
            //     motortest(1,0*//*down*//*);
            //     wobbleClaw.setPosition(*//*open*//*0);
            //     motortest(1,0*//*up*//*);
            //      wobbleClaw.setPosition(*//*close*//*0);
            //   }

            final double now = System.currentTimeMillis();
            if (gamepad1.a &&(now - last_a_press > PRESS_TIME_MS)){
                last_a_press = now;
                shooterOn = !shooterOn;
                if (shooterOn) {
                    pid.start(power, power/2);
                } else {
                    shoot1.setPower(0);
                    shoot2.setPower(0);
                    sleep(5);
                    pid.start(0,0);
                }
            }
            

            pid.loop();

            if (gamepad1.x) {
                shooter.shoot(3);

            }
            if (gamepad1.y) {
                shooter.shoot(1);

            }
            shooter.loop();

/*
        MOVE THESE FUNCTIONS TO DIFFERENT BUTTONS
        if (gamepad1.dpad_down) {
            intake.setPower(1);
        }
        if (gamepad1.dpad_right) {
            intake.setPower((0));
        }
        if (gamepad1.dpad_up) {
            intake.setPower(-1);
        }*/


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

            if (gamepad1.dpad_up) {
                power = power + 0.1;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_down) {
                power = power - 0.1;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_right) {
                power = power + 0.01;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_left) {
                power = power - 0.01;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }


        }

    }

}


