package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "FinalTeleOP")
public class finalTeleOp extends LinearOpMode {

    private Servo flick;
    private Servo stopper;
    private DcMotor intake;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo intakeServo;
    private Servo wobbleClaw;
    private Servo wobbleArm1;
    private Servo wobbleArm2;

    private ShooterStateMachine shooter = new ShooterStateMachine();

    private PID pid = new PID();

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL_1,
        AUTOMATIC_CONTROL_2,
        AUTOMATIC_CONTROL_3,
        AUTOMATIC_CONTROL_4,
        AUTOMATIC_CONTROL_5,
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");
        intakeServo = hardwareMap.servo.get("intakeServo");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        wobbleArm1 = hardwareMap.servo.get("wobbleArm1");
        wobbleArm2 = hardwareMap.servo.get("wobbleArm2");

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        shooter.init(hardwareMap);
        pid.init(hardwareMap);


        // INPUT SHOOTER POWER VALUES HERE
        double whiteLineHighGoalVelocity = 2.10;
        double starterStackHighGoalVelocity = 2.00;
        double powerShotVelocity = 1.60;

        double targetVelocity = whiteLineHighGoalVelocity; // this is the default starting velocity

        boolean a_output = false;
        boolean y_output = false;
        boolean slowModeActive = false;
        boolean back_output = false;
        double gamepadPower = .8;
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
        double teleDeltaTime = 0;
        double teleStartTime = 0;
        final double PRESS_TIME_MS = 200;
        final double WOBBLE_PRESS_TIME_MS = 300;
        final double THREE_SHOT_PRESS_TIME_MS = 400;
        double intakeServoOpen = 0;
        double intakeServoClosed = .20;
        double wobbleClawOpen = .50;
        double wobbleClawClosed = .15;
        double wobbleArmStowed = .12;
        double wobbleArmExtended = .64;
        double wobbleArmUp = .50;
        int powerPreset = 0; // This is for the presets assigned to the START button
        int wobblePreset = 0; // This is for the toggles for the B button

        telemetry.setAutoClear(false);
        Telemetry.Item TeleTargetVelocty = telemetry.addData("Target Velocity:", targetVelocity);
        Telemetry.Item TeleCurrentVelocity = telemetry.addData("Actual Velocity:", pid.currentVelocity);
        Telemetry.Item Power = telemetry.addData("Power:", pid.currentPower);

        intakeServo.setPosition(intakeServoClosed);
        wobbleClaw.setPosition(wobbleClawClosed);
        wobbleArm1.setPosition(wobbleArmStowed);
        wobbleArm2.setPosition(wobbleArmStowed);

        waitForStart();

        while (opModeIsActive()) {

            final double now = System.currentTimeMillis();
            shooter.loop();

            //DRIVETRAIN CONTROL
            switch (currentMode) {
                case DRIVER_CONTROL:
                    float deadZone = (float) 0.5;
                    gamepad1.setJoystickDeadzone(deadZone);
                    motorFrontRight.setPower(-gamepad1.left_stick_y * gamepadPower - gamepad1.right_stick_x * gamepadPower - gamepad1.left_stick_x * gamepadPower);
                    motorBackRight.setPower(-gamepad1.left_stick_y * gamepadPower - gamepad1.right_stick_x * gamepadPower + gamepad1.left_stick_x * gamepadPower);
                    motorFrontLeft.setPower(-gamepad1.left_stick_y * gamepadPower + gamepad1.right_stick_x * gamepadPower + gamepad1.left_stick_x * gamepadPower);
                    motorBackLeft.setPower(-gamepad1.left_stick_y * gamepadPower + gamepad1.right_stick_x * gamepadPower - gamepad1.left_stick_x * gamepadPower);

                    // Y BUTTON SLOWS MOTORS
                    if (gamepad1.y && (now - last_y_press > PRESS_TIME_MS)) {
                        last_y_press = now;
                        slowModeActive = !slowModeActive;
                    }

                    if (slowModeActive) {
                        gamepadPower = .5;
                    } else gamepadPower = .8;


                    // THIS TURNS THE START BUTTON INTO AN AUTOMATED POWER SHOT ROUTINE
                    if (gamepad1.start && (now - last_start_press > PRESS_TIME_MS)) {
                        last_start_press = now;
                        currentMode = Mode.AUTOMATIC_CONTROL_1;
                    }
                    break;
                case AUTOMATIC_CONTROL_1:
                    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    Trajectory powerShot1 = drive.trajectoryBuilder(startPose)
                            .strafeRight(30)
                            .build();
                    drive.followTrajectory(powerShot1);
                    shooter.shoot(1);
                    teleStartTime = System.currentTimeMillis();
                    currentMode = Mode.AUTOMATIC_CONTROL_2;
                    break;
                case AUTOMATIC_CONTROL_2:
                    teleDeltaTime = System.currentTimeMillis() - teleStartTime;
                    if (teleDeltaTime > 300) {
                        currentMode = Mode.AUTOMATIC_CONTROL_3;
                    }
                    break;
                case AUTOMATIC_CONTROL_3:
                    startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    Trajectory powerShot2 = drive.trajectoryBuilder(startPose)
                            .strafeRight(12)
                            .build();
                    drive.followTrajectory(powerShot2);
                    shooter.shoot(1);
                    teleStartTime = System.currentTimeMillis();
                    currentMode = Mode.AUTOMATIC_CONTROL_4;
                    break;
                case AUTOMATIC_CONTROL_4:
                    teleDeltaTime = System.currentTimeMillis() - teleStartTime;
                    if (teleDeltaTime > 300) {
                        currentMode = Mode.AUTOMATIC_CONTROL_5;
                    }
                    break;
                case AUTOMATIC_CONTROL_5:
                    startPose = new Pose2d(0, 0, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    Trajectory powerShot3 = drive.trajectoryBuilder(startPose)
                            .strafeRight(12)
                            .build();
                    drive.followTrajectory(powerShot3);
                    shooter.shoot(1);
                    currentMode = Mode.DRIVER_CONTROL;
                    break;
            }

            //INTAKE
            double intake_power = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(intake_power);

            //SHOOTER ON AND OFF
            if (gamepad1.x && (now - last_x_press > PRESS_TIME_MS)) {
                last_x_press = now;
                pid.shooterOn = !pid.shooterOn;
                if (pid.shooterOn) {
                    pid.start(targetVelocity);
                } else {
                    pid.shoot1.setPower(0);
                    pid.shoot2.setPower(0);
                    sleep(5);
                    pid.setTargetVelocity(0);
                }
            }

            //RUN PID IF THE SHOOTER IS ON
            if (pid.shooterOn) {
                pid.loop();
                TeleCurrentVelocity.setValue(pid.currentVelocity);
            }

            //WOBBLE GOAL ARM
            if (gamepad1.b && (now - last_b_press > WOBBLE_PRESS_TIME_MS)) {
                last_b_press = now;
                wobblePreset++;
                if (wobblePreset == 1) {
                    wobbleArm1.setPosition(wobbleArmExtended);
                    wobbleArm2.setPosition(wobbleArmExtended);
                } else if (wobblePreset == 2) {
                    wobbleArm1.setPosition(wobbleArmStowed);
                    wobbleArm2.setPosition(wobbleArmStowed);
                } else if (wobblePreset == 3) {
                    wobbleArm1.setPosition(wobbleArmUp);
                    wobbleArm2.setPosition(wobbleArmUp);
                    wobblePreset = 0;
                }
            }

            //WOBBLE GOAL CLAW
            if (gamepad1.a && (now - last_a_press > WOBBLE_PRESS_TIME_MS)) {
                last_a_press = now;
                a_output = !a_output;
                wobbleClaw.setPosition(a_output ? wobbleClawOpen : wobbleClawClosed);
            }

            //SHOOT 3 SHOTS
            if (gamepad1.right_bumper && (now - last_rb_press > THREE_SHOT_PRESS_TIME_MS)) {
                last_rb_press = now;
                shooter.shoot(3);
            }

            //SHOOT 1 SHOT
            if (gamepad1.left_bumper && (now - last_lb_press > PRESS_TIME_MS)) {
                last_lb_press = now;
                shooter.shoot(1);
            }

            //ADJUST SHOOTER VELOCITY
            if (gamepad1.dpad_up && (now - last_dpad_up_press > PRESS_TIME_MS)) {
                last_dpad_up_press = now;
                targetVelocity = targetVelocity + 0.1;
                pid.setTargetVelocity(targetVelocity);
            }
            if (gamepad1.dpad_down && (now - last_dpad_down_press > PRESS_TIME_MS)) {
                last_dpad_down_press = now;
                targetVelocity = targetVelocity - 0.1;
                pid.setTargetVelocity(targetVelocity);
            }
            if (gamepad1.dpad_right && (now - last_dpad_right_press > PRESS_TIME_MS)) {
                last_dpad_right_press = now;
                targetVelocity = targetVelocity + 0.01;
                pid.setTargetVelocity(targetVelocity);
            }
            if (gamepad1.dpad_left && (now - last_dpad_left_press > PRESS_TIME_MS)) {
                last_dpad_left_press = now;
                targetVelocity = targetVelocity - 0.01;
                pid.setTargetVelocity(targetVelocity);
            }

            TeleTargetVelocty.setValue((Math.round(100 * targetVelocity)));

            // VELOCITY TOGGLE
           /* if (gamepad1.start && (now - last_start_press > PRESS_TIME_MS)) {
                last_start_press = now;
                powerPreset++;
                if (powerPreset == 1) {
                    targetVelocity = whiteLineHighGoalVelocity;
                    TeleTargetVelocty.setValue((Math.round(100 * targetVelocity)) + "WHITE LINE HIGH GOAL");
                } else if (powerPreset == 2) {
                    targetVelocity = starterStackHighGoalVelocity;
                    TeleTargetVelocty.setValue((Math.round(100 * targetVelocity)) + "STARTER STACK HIGH GOAL");
                } else if (powerPreset == 3) {
                    targetVelocity = powerShotVelocity;
                    powerPreset = 0;
                    TeleTargetVelocty.setValue((Math.round(100 * targetVelocity)) + "POWER SHOT");
                }
            }*/

            // FAILSAFE: if the intake fails to drop during Autonomous, this lets us manually drop it
            if (gamepad1.back && (now - last_back_press > PRESS_TIME_MS)) {
                last_back_press = now;
                back_output = !back_output;
                intakeServo.setPosition(back_output ? intakeServoOpen : intakeServoClosed);
            }
            telemetry.update();
        }
    }
}


