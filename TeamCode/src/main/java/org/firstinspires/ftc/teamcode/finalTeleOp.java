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

    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo flick;
    private Servo stopper;
    private DcMotor intake;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor wobbleMotor;
    private Servo wobbleClaw;

    private ShooterStateMachine shooter = new ShooterStateMachine();

    private final int NUM_PID_ADJUSTMENTS = 10;
    private final int MS_BTWN_VEL_READINGS = 12;
    private final int NUM_VELOCITY_READINGS = 25;

    private double targetVelocity = 0.0;
    private double targetPower = 0.0;
    private long async_prevTime = 0;
    private double async_motorPrev = 0.0;
    private double async_prevError;
    private double velocity_accumulator = 0.0;
    private int velocity_reading_count = 0;
    private int pid_adjust_count = 0;

    private enum PID_STATE {
        RUNNING,
        DONE,
    }

    ;
    private finalTeleOp.PID_STATE state1;

    public void start(double inTargetVelocity, double inPower) {
        state1 = finalTeleOp.PID_STATE.RUNNING;

        targetVelocity = inTargetVelocity;
        targetPower = inPower;
        async_prevTime = System.currentTimeMillis();
        async_motorPrev = -shoot1.getCurrentPosition();
        async_prevError = 0;
        velocity_accumulator = 0.0;
        velocity_reading_count = 0;
        pid_adjust_count = 0;
    }

    public void update() {
        // Read velocity and calculate error; set motors
        long currentTime = System.currentTimeMillis();
        if (currentTime - async_prevTime < MS_BTWN_VEL_READINGS) {
            return;
        }

        // calculate velocity
        final int currentMotor = shoot1.getCurrentPosition();
        final double changeMotor = currentMotor - async_motorPrev;
        final long changeTime = currentTime - async_prevTime;
        final double new_velocity = changeMotor / changeTime;
        velocity_reading_count++;
        velocity_accumulator += new_velocity;

        if (velocity_reading_count < NUM_VELOCITY_READINGS) {
            return;
        }

        final double currentVelocity = velocity_accumulator / velocity_reading_count;
        velocity_reading_count = 0;
        velocity_accumulator = 0.0;

        final double kp = 0.15;
        final double kd = 0.00;
        double error = targetVelocity - currentVelocity;
        final double p = kp * error;
        final double d = kd * ((error - async_prevError) / changeTime);
        shoot1.setPower(p + d + targetPower);
        shoot2.setPower(p + d + targetPower);

        // Update Telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("power", p + d + targetPower);
        dashboardTelemetry.addData("p", p);
        dashboardTelemetry.addData("d", d);
        dashboardTelemetry.addData("velocity", currentVelocity);
        dashboardTelemetry.update();

        // Update state
        targetPower = p + d + targetPower;
        async_prevError = error;
        async_prevTime = currentTime;
        async_motorPrev = currentMotor;

        if (pid_adjust_count++ >= NUM_PID_ADJUSTMENTS) {
            state1 = finalTeleOp.PID_STATE.DONE;
        }
    }

    public boolean done() {
        return state1 == finalTeleOp.PID_STATE.DONE;
    }


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

        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        wobbleMotor = hardwareMap.dcMotor.get("wobbleMotor");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot1.setDirection(DcMotor.Direction.REVERSE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);
        shooter.init();

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
                    shoot1.setPower(.65);
                    shoot2.setPower(.65);
                } else {
                    shoot1.setPower(0);
                    shoot2.setPower(0);
                }
            }


            if (gamepad1.x) {
                shooter.shoot(3);
                shooter.loop();
            }
            if (gamepad1.y) {
                shooter.shoot(1);
                shooter.loop();
            }

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


