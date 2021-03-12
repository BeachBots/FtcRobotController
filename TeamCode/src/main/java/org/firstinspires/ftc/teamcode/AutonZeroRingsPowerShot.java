package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.opencv.core.Core.inRange;

@Config
@Autonomous(name = "AutonZeroRingsPowerShot", group = "LinearOpmode")
public class AutonZeroRingsPowerShot extends LinearOpMode {

    private Servo flick;
    private Servo stopper;
    private DcMotor intake;
    private Servo intakeServo;
    private Servo wobbleClaw;
    private Servo wobbleArm1;
    private Servo wobbleArm2;

    private ShooterStateMachine shooter = new ShooterStateMachine();
    private PID pid = new PID();

    private ZERO_RINGS_STATE state;
    private ONE_RINGS_STATE state2;
    // private FOUR_RINGS_STATE state3;

    private long autonDeltaTime = System.currentTimeMillis();
    private long autonStartTime;

    private double shooterLineVelocity = 2.10;
    private double starterStackVelocity = 1.74;
    private double powerShotVelocity = 1.53;

    public enum ZERO_RINGS_STATE {
        INITIALIZING,  // drop intake, start shooter, start moving to line
        POWERSHOT_1,
        POWERSHOT_2,
        POWERSHOT_2_FIRE,
        POWERSHOT_3,
        POWERSHOT_3_FIRE,
        DRIVE_TO_SQUARE_A,
        DROP_WOBBLE_1,
        INTAKE,
        DRIVE_TO_LINE,
        SHOOT_RINGS, // reach line and fire three shots
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        WAIT,
        WAIT_2,
        MOVE_TO_DROP_WOBBLE_2,
        DROP_WOBBLE_2,
        STOW_ARM,
        TURN_TO_PARK,
        DONE,
    }

    public enum ONE_RINGS_STATE {

        INITIALIZING,
        DRIVE_TO_LINE,
        DRIVE_TO_SQUARE_B,
        DROP_WOBBLE_1,
        WAIT,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        WAIT_2,
        INTAKE_STARTER_STACK,
        WAIT_3,
        RETURN_TO_LINE,
        SHOOT_RING,
        RETURN_TO_SQUARE_B,
        DROP_WOBBLE_2,
        STOW_ARM,
        PARK,
        DONE,
    }

    public static int VALUE_0A = 0;
    public static int VALUE_0B = 25;
    public static int VALUE_1A = 100;
    public static int VALUE_1B = 255;
    public static int VALUE_2A = 80;
    public static int VALUE_2B = 255;
    public static int val = 1;
    OpenCvCamera webcam;

    double[] bottomRollingAvg = {0, 0, 0};
    double bottomAvg = 0;
    int stack;
    boolean objectDetected = false;


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //This is for all hardware not identified in PID or SHOOTERSTATEMACHINE
        intake = hardwareMap.dcMotor.get("intake");
        intakeServo = hardwareMap.servo.get("intakeServo");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        wobbleArm1 = hardwareMap.servo.get("wobbleArm1");
        wobbleArm2 = hardwareMap.servo.get("wobbleArm2");

        intake.setDirection(DcMotor.Direction.REVERSE);

        double intakeServoOpen = 0;
        double intakeServoClosed = .20;
        double wobbleClawOpen = .65; // was .50;
        double wobbleClawClosed = .15;
        double wobbleArmStowed = .12;
        double wobbleArmExtended = .64;
        double wobbleArmUp = .55;

        wobbleClaw.setPosition(wobbleClawClosed);
        wobbleArm1.setPosition(wobbleArmStowed);
        wobbleArm2.setPosition(wobbleArmStowed);
        intakeServo.setPosition(intakeServoClosed);

        // ======  OPENCV  ======

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.setPipeline(new stackPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        while (isStarted() == false) {

            telemetry.addData("Bottom Avg", bottomAvg);
            telemetry.addData("Stack of", stack);
            telemetry.update();

        }
        webcam.stopStreaming();

        // ====== END OF OPEN CV ======

        shooter.init(hardwareMap);
        pid.init(hardwareMap);

        // This identifies the starting position of our robot -- otherwise it default to (0,0) which is the center of the field.

        Pose2d startPose = new Pose2d(-63, -62.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // This is where we build all the trajectories. We won't call them until the bottom.

        //zero rings

        Trajectory zeroRings1 = drive.trajectoryBuilder(startPose) // Move to First PowerShot
               .splineToConstantHeading(new Vector2d(-10, -4), Math.toRadians(0))
               .build();

        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end()) // Move to Second PowerShot
                .strafeTo (new Vector2d(-10, -16))
                .build();

        Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end()) // Move to Third PowerShot
                .strafeTo (new Vector2d(-10, -26))
                .build();

        Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end()) // Move to Square A
                .lineToLinearHeading(new Pose2d(8, -45, Math.toRadians(90)))
                .build();

        Trajectory zeroRings5 = drive.trajectoryBuilder(zeroRings4.end()) // Spline to front
                .splineToConstantHeading(new Vector2d(67, -56), Math.toRadians(90))
                .build();

        Trajectory zeroRings6 = drive.trajectoryBuilder(zeroRings5.end()) // Intake
        .lineToLinearHeading(
                new Pose2d(67, 5, Math.toRadians(90)),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
        )
                .build();

        Trajectory zeroRings7 = drive.trajectoryBuilder(zeroRings6.end()) // Move to shooting line
                .lineToLinearHeading(new Pose2d(-4, -38, Math.toRadians(0)))
                .build();

        Trajectory zeroRings8 = drive.trajectoryBuilder(zeroRings7.end()) // Move to Wobble 2
                .lineToLinearHeading(
                        new Pose2d(-35, -17, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory zeroRings9 = drive.trajectoryBuilder(zeroRings8.end()) // Move to Square A
                .lineToLinearHeading(
                        new Pose2d(8, -43, Math.toRadians(90)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        //one ring

        Trajectory oneRing1 = drive.trajectoryBuilder(startPose)  // Move to shooting line
                .splineToConstantHeading(new Vector2d(-30, -58), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-10, -39), Math.toRadians(0))
                .build();

        Trajectory oneRing2 = drive.trajectoryBuilder(oneRing1.end()) // Move to Square B
                .lineToLinearHeading(new Pose2d(28, -24, Math.toRadians(90)))
                .build();

        Trajectory oneRing3 = drive.trajectoryBuilder(oneRing2.end())  // Move to Wobble 2
                .lineToLinearHeading(
                        new Pose2d(-44, -17, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory oneRing4 = drive.trajectoryBuilder(oneRing3.end()) // Move to starter stack
                .splineTo(
                        new Vector2d(-32, -25), Math.toRadians(-60),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        Trajectory oneRing5 = drive.trajectoryBuilder(oneRing4.end()) // Move to shooting line
//                .lineToLinearHeading(new Pose2d(-10, -39, Math.toRadians(0)))
//                .build();
                .lineToLinearHeading(
                        new Pose2d(-10, -39, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory oneRing6 = drive.trajectoryBuilder(oneRing5.end()) // Move to Square B
                .lineToLinearHeading(new Pose2d(18, -19, Math.toRadians(90)))
                .build();

        Trajectory oneRing7 = drive.trajectoryBuilder(oneRing6.end()) // Move to white line
                .lineToLinearHeading(new Pose2d(6, -18, Math.toRadians(0)))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        if (stack == 0) {
            state = ZERO_RINGS_STATE.INITIALIZING;
            while (opModeIsActive()) {
                switch (state) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(zeroRings1);  // Kick off moving to POWERSHOT 1
                        pid.start(powerShotVelocity);
                        state = ZERO_RINGS_STATE.POWERSHOT_1;
                        break;
                    case POWERSHOT_1:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy() || !pid.ready()) { // Still moving to position
                            drive.update();
                            pid.loop();
                        } else if (pid.ready()) {
                            shooter.shoot(1);
                            state = ZERO_RINGS_STATE.POWERSHOT_2;
                        }
                        break;
                    case POWERSHOT_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            drive.followTrajectoryAsync(zeroRings2);
                            state = ZERO_RINGS_STATE.POWERSHOT_2_FIRE;
                        }
                        break;
                    case POWERSHOT_2_FIRE:
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            shooter.shoot(1);
                            state = ZERO_RINGS_STATE.POWERSHOT_3;
                        }
                        break;
                    case POWERSHOT_3:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            drive.followTrajectoryAsync(zeroRings3);
                            state = ZERO_RINGS_STATE.POWERSHOT_3_FIRE;
                        }
                        break;
                    case POWERSHOT_3_FIRE:
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            shooter.shoot(1);
                            state = ZERO_RINGS_STATE.DRIVE_TO_SQUARE_A;
                        }
                        break;
                    case DRIVE_TO_SQUARE_A:
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            pid.shoot1.setPower(0);
                            pid.shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            drive.followTrajectoryAsync(zeroRings4);
                            wobbleArm1.setPosition(wobbleArmUp);
                            wobbleArm2.setPosition(wobbleArmUp);
                            state = ZERO_RINGS_STATE.DROP_WOBBLE_1;
                        }
                        break;
                    case DROP_WOBBLE_1:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            wobbleClaw.setPosition(wobbleClawOpen);
                            drive.followTrajectoryAsync(zeroRings5);
                            state = ZERO_RINGS_STATE.INTAKE;
                        }
                        break;
                    case INTAKE:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            intake.setPower(1);
                            drive.followTrajectoryAsync(zeroRings6);
                            state = ZERO_RINGS_STATE.DRIVE_TO_LINE;
                        }
                        break;
                    case DRIVE_TO_LINE:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            pid.start(shooterLineVelocity);
                            drive.followTrajectoryAsync(zeroRings7);
                            state = ZERO_RINGS_STATE.SHOOT_RINGS;
                        }
                        break;
                    case SHOOT_RINGS:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy() || !pid.ready()) { // Still moving to shooting line.
                            drive.update();
                            pid.loop();
                        } else if (pid.ready()) {
                            intake.setPower(0);
                            shooter.shoot(3);
                            state = ZERO_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                        }
                        break;
                    case DRIVE_TO_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            pid.shoot1.setPower(0);
                            pid.shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            drive.followTrajectoryAsync(zeroRings8);
                            wobbleArm1.setPosition(wobbleArmExtended);
                            wobbleArm2.setPosition(wobbleArmExtended);
                            state = ZERO_RINGS_STATE.PICK_UP_WOBBLE_2;
                        }
                        break;
                    case PICK_UP_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            autonStartTime = System.currentTimeMillis();
                            state = ZERO_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 700) {
                            wobbleClaw.setPosition(wobbleClawClosed); // CLOSE CLAW
                            autonStartTime = System.currentTimeMillis();
                            state = ZERO_RINGS_STATE.WAIT_2;
                        }
                        break;
                    case WAIT_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 800) {
                            wobbleArm1.setPosition(wobbleArmUp);
                            wobbleArm2.setPosition(wobbleArmUp);
                            state = ZERO_RINGS_STATE.MOVE_TO_DROP_WOBBLE_2;
                        }
                        break;
                    case MOVE_TO_DROP_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        drive.followTrajectoryAsync(zeroRings9);  // MOVE TO BOX A TO DROP OFF WOBBLE 2
                        state = ZERO_RINGS_STATE.DROP_WOBBLE_2;
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            wobbleClaw.setPosition(wobbleClawOpen);
                            state = ZERO_RINGS_STATE.STOW_ARM;
                        }
                        break;
                    case STOW_ARM:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        wobbleArm1.setPosition(wobbleArmStowed);
                        wobbleArm2.setPosition(wobbleArmStowed);
                        state = ZERO_RINGS_STATE.TURN_TO_PARK;
                        break;
                    case TURN_TO_PARK:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        drive.turn(Math.toRadians(-90));
                        state = ZERO_RINGS_STATE.DONE;
                        break;
                    case DONE:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        break;
                    default:
                }
            }
        }

        if (stack == 1) {
            state2 = ONE_RINGS_STATE.INITIALIZING;
            while (opModeIsActive()) {
                pid.loop();
                switch (state2) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(oneRing1);
                        pid.start(shooterLineVelocity);
                        state2 = ONE_RINGS_STATE.DRIVE_TO_LINE;
                        break;
                    case DRIVE_TO_LINE:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (drive.isBusy() || !pid.ready()) { // Still moving to shooting line.
                            drive.update();
                            pid.loop();
                        } else if (pid.ready()) {
                            shooter.shoot(3);
                            state2 = ONE_RINGS_STATE.DRIVE_TO_SQUARE_B;
                        }
                        break;
                    case DRIVE_TO_SQUARE_B:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            pid.shoot1.setPower(0);
                            pid.shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            drive.followTrajectoryAsync(oneRing2);
                            wobbleArm1.setPosition(wobbleArmUp);
                            wobbleArm2.setPosition(wobbleArmUp);
                            state2 = ONE_RINGS_STATE.DROP_WOBBLE_1;
                        }
                        break;
                    case DROP_WOBBLE_1:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (drive.isBusy()) { // Still moving to Square B
                            drive.update();
                        } else {
                            wobbleClaw.setPosition(wobbleClawOpen);
                            autonStartTime = System.currentTimeMillis();
                            state2 = ONE_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 700) {
                            state2 = ONE_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                        }
                        break;
                    case DRIVE_TO_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectoryAsync(oneRing3);
                        wobbleArm1.setPosition(wobbleArmExtended);
                        wobbleArm2.setPosition(wobbleArmExtended);
                        state2 = ONE_RINGS_STATE.PICK_UP_WOBBLE_2;
                        break;
                    case PICK_UP_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            autonStartTime = System.currentTimeMillis();
                            state2 = ONE_RINGS_STATE.WAIT_2;
                        }
                        break;
                    case WAIT_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 800) {
                            wobbleClaw.setPosition(wobbleClawClosed);
                            state2 = ONE_RINGS_STATE.INTAKE_STARTER_STACK;
                        }
                        break;
                    case INTAKE_STARTER_STACK:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        intake.setPower(.8);  // was 1
                        pid.start(shooterLineVelocity);
                        drive.followTrajectoryAsync(oneRing4);
                        autonStartTime = System.currentTimeMillis();
                        state2 = ONE_RINGS_STATE.WAIT_3;
                        break;
                    case WAIT_3:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 2000) {
                            state2 = ONE_RINGS_STATE.RETURN_TO_LINE;
                        }
                        break;
                    case RETURN_TO_LINE:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            drive.followTrajectoryAsync(oneRing5);
                            state2 = ONE_RINGS_STATE.SHOOT_RING;
                        }
                        break;
                    case SHOOT_RING:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (drive.isBusy() || !pid.ready()) {
                            drive.update();
                            pid.loop();
                        } else if (pid.ready()) {
                            shooter.shoot(3);
                            state2 = ONE_RINGS_STATE.RETURN_TO_SQUARE_B;
                        }
                        break;
                    case RETURN_TO_SQUARE_B:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            intake.setPower(0);
                            pid.shoot1.setPower(0);
                            pid.shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            wobbleArm1.setPosition(wobbleArmUp);
                            wobbleArm2.setPosition(wobbleArmUp);
                            drive.followTrajectory(oneRing6);
                            state2 = ONE_RINGS_STATE.DROP_WOBBLE_2;
                        }
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        wobbleClaw.setPosition(wobbleClawOpen);
//                        autonStartTime = System.currentTimeMillis();
//                        state2 = ONE_RINGS_STATE.STOW_ARM;
//                        break;
//                    case STOW_ARM:
//                        telemetry.addData("state = ", state2);
//                        telemetry.update();
//                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
//                        if (autonDeltaTime > 800) {
                        wobbleArm1.setPosition(wobbleArmStowed);
                        wobbleArm2.setPosition(wobbleArmStowed);
                        state2 = ONE_RINGS_STATE.PARK;
                        //                       }
                        break;
                    case PARK:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing7);
                        state2 = ONE_RINGS_STATE.DONE;
                        break;
                    case DONE:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        break;
                    default:
                }

            }

        }


    }

    class stackPipeline extends OpenCvPipeline {

        Mat submat;

        Mat greyBottom = new Mat();
        Mat matTest = new Mat();

        @Override
        public void init(Mat firstFrame) {
            submat = firstFrame.submat(0, 240, 0, 320);
        }

        @Override
        public Mat processFrame(Mat input) {
            greyBottom = input.submat(100, 240, 0, 200);

            Imgproc.cvtColor(greyBottom, greyBottom, Imgproc.COLOR_RGB2HSV);
            Scalar lowerOrange = new Scalar(VALUE_0A, VALUE_1A, VALUE_2A);
            Scalar upperOrange = new Scalar(VALUE_0B, VALUE_1B, VALUE_2B);
            inRange(greyBottom, lowerOrange, upperOrange, greyBottom);

            List<MatOfPoint> test = new ArrayList<MatOfPoint>();

            Imgproc.findContours(greyBottom, test, matTest, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            objectDetected = false;

            bottomRollingAvg = new double[]{0, 0, 0};
            bottomAvg = (int) Core.mean(greyBottom).val[0];

            if (bottomAvg < 7) {
                stack = 0;
            } else if ((bottomAvg > 7) && (bottomAvg < 16)) {
                stack = 1;
            } else if (bottomAvg > 16) {
                stack = 4;
            }
            if (val == 0) {
                return input;
            } else {
                return greyBottom;
            }

        }

    }

}