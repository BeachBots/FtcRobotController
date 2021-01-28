package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.ShooterStateMachine;
import org.firstinspires.ftc.teamcode.Webcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.Core.inRange;

@Autonomous(name = "FinalAuton", group = "LinearOpmode")

public class FinalAuton extends LinearOpMode {

    private Servo flick;
    private DcMotor shoot1;
    private DcMotor shoot2;
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
    private FOUR_RINGS_STATE state3;

    private long autonDeltaTime = System.currentTimeMillis();
    private long autonStartTime;

    private double shooterLineVelocity = 1.80;
    private double starterStackVelocity = 2.00;

    public enum ZERO_RINGS_STATE {
        INITIALIZING,  // drop intake, start shooter, start moving to line
        DRIVE_TO_LINE, // reach line and fire three shots
        DRIVE_RIGHT, // drive to Box A
        DROP_WOBBLE_1,
        WAIT,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        MOVE_TO_DROP_WOBBLE_2,
        DROP_WOBBLE_2,
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
        RETURN_TO_SQUARE_B,
        DROP_WOBBLE_2,
        PARK,
        DONE,
    }

    public enum FOUR_RINGS_STATE {
        INITIALIZING,
        DRIVE_TO_LINE,
        DRIVE_TO_SQUARE_C,
        DROP_WOBBLE_1,
        WAIT,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        RETURN_TO_SQUARE_C,
        DROP_WOBBLE_2,
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
        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");

        intake.setDirection(DcMotor.Direction.REVERSE);

        double intakeServoOpen = 0;
        double intakeServoClosed = .20;
        double wobbleClawOpen = .50;
        double wobbleClawClosed = .15;
        double wobbleArmStowed = .10;
        double wobbleArmExtended = .64;
        double wobbleArmUp = .29;

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

            telemetry.addData("bottom Avg", bottomAvg);
            telemetry.addData("Stack of:", stack);
            telemetry.update();

        }
        webcam.stopStreaming();

        // ====== END OF OPEN CV ======

        shooter.init(hardwareMap);
        pid.init(hardwareMap);

        // This identifies the starting position of our robot -- otherwise it default to (0,0) which is the center of the field.

        Pose2d startPose = new Pose2d(-63, -34.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // This is where we build all the trajectories. We won't call them until the bottom.

        //zero rings

        Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-4, -45), Math.toRadians(0))  // Move to shooting line
                .build();

        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                .lineToLinearHeading(new Pose2d(4, -52, Math.toRadians(90)))  // Move to Square A
                .build();

        Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
                .lineToLinearHeading(new Pose2d(-37.5, -45, Math.toRadians(0))) // Move to get Wobble #2
                .build();

        Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
                .lineToLinearHeading(new Pose2d(7, -45, Math.toRadians(90))) // Move to Square A
                .build();


        //one ring

        Trajectory oneRing1 = drive.trajectoryBuilder(startPose)  // Move to shooting line
                .splineToConstantHeading(new Vector2d(-4, -45), Math.toRadians(0))
                .build();

        Trajectory oneRing2 = drive.trajectoryBuilder(oneRing1.end()) // Move to Square B
                .lineToLinearHeading(new Pose2d(35, -28, Math.toRadians(90)))
                .build();

        Trajectory oneRing3 = drive.trajectoryBuilder(oneRing2.end())  // Move to Wobble 2
                .lineToLinearHeading(new Pose2d(-37.5, -45, Math.toRadians(0)))
                .build();

        Trajectory oneRing4 = drive.trajectoryBuilder(oneRing3.end()) // Move to Square B
                .lineToLinearHeading(new Pose2d(28, -25, Math.toRadians(90)))
                .build();

        Trajectory oneRing5 = drive.trajectoryBuilder(oneRing4.end()) // Move to white line
                .lineToLinearHeading(new Pose2d(9, -18, Math.toRadians(0)))
                .build();


        //four rings

        Trajectory fourRing1 = drive.trajectoryBuilder(startPose) // Move to shooting line
                .splineToConstantHeading(new Vector2d(-4, -45), Math.toRadians(0))
                .build();

        Trajectory fourRing2 = drive.trajectoryBuilder(fourRing1.end())  // Move to Square C
                .lineToLinearHeading(new Pose2d(60, -52, Math.toRadians(90)))
                .build();

        Trajectory fourRing3 = drive.trajectoryBuilder(fourRing2.end())  // Move to Wobble 2
                .lineToLinearHeading(new Pose2d(-37.5, -45, Math.toRadians(0)))
                .build();

        Trajectory fourRing4 = drive.trajectoryBuilder(fourRing3.end())  // Return to Square C
                .lineToLinearHeading(new Pose2d(53, -49, Math.toRadians(90)))
                .build();

        Trajectory fourRing5 = drive.trajectoryBuilder(fourRing4.end()) // Move to white line
                .lineToLinearHeading(new Pose2d(9, -28, Math.toRadians(0)))
                .build();


        waitForStart();
        if (
                isStopRequested()) return;


        if (stack == 0) {
            state = ZERO_RINGS_STATE.INITIALIZING;
            while (opModeIsActive()) {
                switch (state) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(zeroRings1);  // Kick off moving to SHOOTING LINE
                        pid.start(shooterLineVelocity);
                        state = ZERO_RINGS_STATE.DRIVE_TO_LINE;
                        break;
                    case DRIVE_TO_LINE:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy() || !pid.ready()) { // Still moving to shooting line.
                            drive.update();
                            pid.loop();
                        } else if (pid.ready()) {
                            shooter.shoot(3);
                            state = ZERO_RINGS_STATE.DRIVE_RIGHT;
                        }
                        break;
                    case DRIVE_RIGHT:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            shoot1.setPower(0);
                            shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            drive.followTrajectoryAsync(zeroRings2);
                            wobbleArm1.setPosition(wobbleArmExtended);
                            wobbleArm2.setPosition(wobbleArmExtended);
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
                            wobbleArm1.setPosition(wobbleArmStowed);
                            wobbleArm2.setPosition(wobbleArmStowed);
                            autonStartTime = System.currentTimeMillis();
                            state = ZERO_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 1000) {
                            state = ZERO_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                        }
                        break;
                    case DRIVE_TO_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        drive.followTrajectoryAsync(zeroRings3);  // MOVE TO PICK UP WOBBLE 2
                        wobbleArm1.setPosition(wobbleArmExtended);
                        wobbleArm2.setPosition(wobbleArmExtended);
                        state = ZERO_RINGS_STATE.PICK_UP_WOBBLE_2;
                        break;
                    case PICK_UP_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            wobbleClaw.setPosition(wobbleClawClosed); // CLOSE CLAW
                            state = ZERO_RINGS_STATE.MOVE_TO_DROP_WOBBLE_2;
                        }
                        break;
                    case MOVE_TO_DROP_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        drive.followTrajectory(zeroRings4);  // MOVE TO BOX A TO DROP OFF WOBBLE 2
                        state = ZERO_RINGS_STATE.DROP_WOBBLE_2;
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        wobbleClaw.setPosition(wobbleClawOpen);
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
                            shoot1.setPower(0);
                            shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            drive.followTrajectoryAsync(oneRing2);
                            wobbleArm1.setPosition(wobbleArmExtended);
                            wobbleArm2.setPosition(wobbleArmExtended);
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
                            wobbleArm1.setPosition(wobbleArmStowed);
                            wobbleArm2.setPosition(wobbleArmStowed);
                            autonStartTime = System.currentTimeMillis();
                            state2 = ONE_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 1000) {
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
                            wobbleClaw.setPosition(wobbleClawClosed);
                            state2 = ONE_RINGS_STATE.RETURN_TO_SQUARE_B;
                        }
                        break;
                    case RETURN_TO_SQUARE_B:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing4);
                        state2 = ONE_RINGS_STATE.DROP_WOBBLE_2;
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        wobbleClaw.setPosition(wobbleClawOpen);
                        wobbleArm1.setPosition(wobbleArmStowed);
                        wobbleArm2.setPosition(wobbleArmStowed);
                        state2 = ONE_RINGS_STATE.PARK;
                        break;
                    case PARK:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing5);
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

        if (stack == 4) {
            state3 = FOUR_RINGS_STATE.INITIALIZING;
            while (opModeIsActive()) {
                pid.loop();
                switch (state3) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(fourRing1);
                        pid.start(shooterLineVelocity);
                        state3 = FOUR_RINGS_STATE.DRIVE_TO_LINE;
                        break;
                    case DRIVE_TO_LINE:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        if (drive.isBusy() || !pid.ready()) { // Still moving to shooting line.
                            drive.update();
                            pid.loop();
                        } else if (pid.ready()) {
                            shooter.shoot(3);
                            state3 = FOUR_RINGS_STATE.DRIVE_TO_SQUARE_C;
                        }
                        break;
                    case DRIVE_TO_SQUARE_C:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            shoot1.setPower(0);
                            shoot2.setPower(0);
                            sleep(5);
                            pid.start(0);
                            drive.followTrajectoryAsync(fourRing2);
                            wobbleArm1.setPosition(wobbleArmExtended);
                            wobbleArm2.setPosition(wobbleArmExtended);
                            state3 = FOUR_RINGS_STATE.DROP_WOBBLE_1;
                        }
                        break;
                    case DROP_WOBBLE_1:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        if (drive.isBusy()) { // Still moving to Square C
                            drive.update();
                        } else {
                            wobbleClaw.setPosition(wobbleClawOpen);
                            wobbleArm1.setPosition(wobbleArmStowed);
                            wobbleArm2.setPosition(wobbleArmStowed);
                            autonStartTime = System.currentTimeMillis();
                            state3 = FOUR_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 1000) {
                            state3 = FOUR_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                        }
                        break;
                    case DRIVE_TO_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectoryAsync(fourRing3);
                        wobbleArm1.setPosition(wobbleArmExtended);
                        wobbleArm2.setPosition(wobbleArmExtended);
                        state3 = FOUR_RINGS_STATE.PICK_UP_WOBBLE_2;
                        break;
                    case PICK_UP_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        if (drive.isBusy()) {
                            drive.update();
                        } else {
                            wobbleClaw.setPosition(wobbleClawClosed);
                            state3 = FOUR_RINGS_STATE.RETURN_TO_SQUARE_C;
                        }
                        break;
                    case RETURN_TO_SQUARE_C:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectory(fourRing4);
                        state3 = FOUR_RINGS_STATE.DROP_WOBBLE_2;
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        wobbleClaw.setPosition(wobbleClawOpen);
                        wobbleArm1.setPosition(wobbleArmStowed);
                        wobbleArm2.setPosition(wobbleArmStowed);
                        state3 = FOUR_RINGS_STATE.PARK;
                        break;
                    case PARK:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectory(fourRing5);
                        state3 = FOUR_RINGS_STATE.DONE;
                        break;
                    case DONE:
                        telemetry.addData("state = ", state3);
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
            greyBottom = input.submat(100, 240, 60, 320);

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