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
    private Servo intakeServo;

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
        intakeServo = hardwareMap.servo.get("intakeServo");

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

        double power = .5;

        flick.setPosition(0.48);
        stopper.setPosition(0.85);
        boolean output = false;  // this is for the intake
        boolean shooterOn = false;
        double last_rb_press = 0.;
        double last_lb_press = 0.;
        double last_a_press = 0.;
        double last_b_press = 0.;
        double last_x_press = 0.;
        double last_y_press = 0.;
        double last_start_press = 0.;
        double last_back_press = 0.;
        double last_dpad_up_press = 0.;
        double last_dpad_down_press = 0.;
        double last_dpad_left_press = 0.;
        double last_dpad_right_press = 0.;
        final double PRESS_TIME_MS = 400;
        double intakeServoOpen = .2;
        double intakeServoClosed = 0;
        double wobbleClawOpen = .8;
        double wobbleClawClosed = .6;
        int powerPreset = 0;

        // INPUT SHOOTER POWER VALUES HERE

        double whiteLineHighGoalPower = .50;
        double starterStackHighGoalPower = .65;
        double powerShotPower = .30;

        //

        waitForStart();

        while (opModeIsActive()) {

            float deadZone = (float) 0.5;
            gamepad1.setJoystickDeadzone(deadZone);

            motorFrontRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);
            motorBackRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorFrontLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorBackLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);


            final double now = System.currentTimeMillis();
            if (gamepad1.x && (now - last_x_press > PRESS_TIME_MS)) {
                last_x_press = now;
                shooterOn = !shooterOn;
                if (shooterOn) {
                    pid.start(power, power / 2);
                } else {
                    shoot1.setPower(0);
                    shoot2.setPower(0);
                    sleep(5);
                    pid.start(0, 0);
                }
            }

            if (shooterOn) {
                pid.loop();
            }

            if (gamepad1.y && (now - last_y_press > PRESS_TIME_MS)) {
                last_y_press = now;
                //USE FOR WOBBLE GOAL
            }

            if (gamepad1.b && (now - last_b_press > PRESS_TIME_MS)) {
                last_b_press = now;
                //USE FOR WOBBLE GOAL
            }

            if (gamepad1.a && (now - last_a_press > PRESS_TIME_MS)) {
                last_a_press = now;
                //USE FOR WOBBLE GOAL
            }

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

            shooter.loop();

            double intake_power = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intake_power);

            if (gamepad1.right_bumper && (now - last_rb_press > PRESS_TIME_MS)) {
                last_rb_press = now;
                output = !output;
                shooter.shoot(3);
            }

            if (gamepad1.left_bumper && (now - last_lb_press > PRESS_TIME_MS)) {
                last_lb_press = now;
                output = !output;
                shooter.shoot(1);
            }

            if (gamepad1.dpad_up && (now - last_dpad_up_press > 200)) {
                last_dpad_up_press = now;
                output = !output;
                power = power + 0.1;
                telemetry.addData("power", (Math.round(100 * power)));
                telemetry.update();
            }

            if (gamepad1.dpad_down && (now - last_dpad_down_press > 200)) {
                last_dpad_down_press = now;
                output = !output;
                power = power - 0.1;
                telemetry.addData("power", (Math.round(100 * power)));
                telemetry.update();
            }

            if (gamepad1.dpad_right && (now - last_dpad_right_press > 200)) {
                last_dpad_right_press = now;
                output = !output;
                power = power + 0.01;
                telemetry.addData("power", (Math.round(100 * power)));
                telemetry.update();
            }

            if (gamepad1.dpad_left && (now - last_dpad_left_press > 200)) {
                last_dpad_left_press = now;
                output = !output;
                power = power - 0.01;
                telemetry.addData("power", (Math.round(100 * power)));
                telemetry.update();
                sleep(200);
            }

            if (gamepad1.start && (now - last_start_press > 300)) {
                last_start_press = now;
                powerPreset++;
                if (powerPreset == 1) {
                    power = whiteLineHighGoalPower;
                    telemetry.addData("WHITE LINE HIGH GOAL : power", (Math.round(100 * power)));
                    telemetry.update();
                } else if (powerPreset == 2) {
                    power = starterStackHighGoalPower;
                    telemetry.addData("STARTER STACK HIGH GOAL : power", (Math.round(100 * power)));
                    telemetry.update();
                } else if (powerPreset == 3) {
                    power = powerShotPower;
                    powerPreset = 0;
                    telemetry.addData("POWER SHOT : power", (Math.round(100 * power)));
                    telemetry.update();
                }
            }

            if (gamepad1.back && (now - last_back_press > PRESS_TIME_MS)) {
                last_back_press = now;
                output = !output;
                intakeServo.setPosition(output ? intakeServoOpen : intakeServoClosed);
            }
        }
    }
}


