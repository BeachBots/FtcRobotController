package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AutonTest;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.ShooterStateMachine;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name = "SampleAuton", group = "LinearOpmode")

public class SampleAuton extends LinearOpMode {

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

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "ATOQy0T/////AAABmQ/SASRSuUzAugTNY1JUz6NSX/K0IyvPgOTMGlPcjAUyxoQlqULPX1jcW4C4fMzALWwznPmVdS4QFyFERfGevgAJPX1U8c6c1wfPTrhaqwhhoG0SBo/8b6iGaeweb65NN1Xu7PG+LHieN8rr339wsfsGROM9TW3oRp6uYdDFq30aM9tAIadifcbtQq9XUSdUyzF7Owgr8QjIbAw57OYIb6Bwl+7tenxUVSM+pYEkZyVdbCWAWEHAeu10tX7qGXmPEENNiKfnTT/TvGCsjTVK0Xa536mx70V74J/wBadTw32md6QMKjQmWWtCqyYnYZenYws42NGZzXsD/cLU+IbXOIJBaohDU8SaPr7mienrXGgc";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean isRunning = true;

    private ZERO_RINGS_STATE state;
    private ONE_RINGS_STATE state2;
    private FOUR_RINGS_STATE state3;

    private long autonDeltaTime = System.currentTimeMillis();
    private long autonStartTime;


    private double targetVelocity = 175;

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


    ;

    /*
    HOW TO TURN YOUR CODE INTO ANOTHER STATE MACHINE:
    1. Declare a state like so, that defines concrete states where different actions will be done.
    2. Inside a while loop like opModeIsActive, make a switch statement
    3. You just take the steps in order, and make a case statement for each stage where you do something.
        switch(state){
          case INITIALIZING:
            drive.followTrajectory(trajectory);
            state = THIS_STATE.NEXT_STEP_IN_SERIES;
            break;
        }
    4. This should just get you behavior you already had - do all things in order once.
    5. One step at a time, don't need to do all of them in one shot
       Change a call to followTrajectory() to followTrajectoryAsync(trajectory);
       And then move that call to when we *enter* the state. i.e. go to the previous state, and
       when you transition to the next one, that's where you call followTrajectoryAsync().
       INITIALIZING should just call the function and immediately move to the next state.
       Any other one should have a condition for moving forward.
    6. In a state where we're supposed to be driving, we check if the trajectory is still being
       followed. This is done by calling if (drive.isBusy()) {drive.update();}
       If we've already completed it, we can enter the next state.
    7. For cases where you just want to wait, instead of starting a trajectory, you save a time
       marker when you enter the next state.
       In the following state, you check "now" vs that saved timestamp until the desired amount
       of time has passed.
    8. If we want to spin up the PID controller or anything else that updates, call pid.update();
       or whatever at the same time as stuff like drive.update() - this is being called continuously
       until a drive is complete.
       You can also check if pid.ready() before ending a state if it's important the PID has reached
       the correct speed before moving on.
    */;


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

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 1.78);
        }

        int rings = 0;

        //detect using camera
        boolean ison = true;

        while (isStarted() == false) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    if (updatedRecognitions.size() == 0) {
                        rings = 0;
                    }
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            rings = 1;
                            isRunning = false;
                        } else if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            rings = 4;
                            isRunning = false;
                        }
                    }
                    telemetry.addData("rings = ", rings);
                    telemetry.update();
                }
            }

        }

        shooter.init(hardwareMap);
        pid.init(hardwareMap);

        // waitForStart(); // THIS WAS HERE BUT WE SHOULD MOVE TO LINE 304

        // This identifies the starting position of our robot -- otherwise it default to (0,0) which is the center of the field.
        // We should tune this number, which I'm estimating to be (-63, -34.5).
        Pose2d startPose = new Pose2d(-63, -34.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // This is where we build all the trajectories. We won't call them until the bottom.

        //zero rings

        Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-3, -45), Math.toRadians(0))  // Move to shooting line
                .build();

        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                .lineToLinearHeading(new Pose2d(4, -52, Math.toRadians(90)))  // Move to Square A
                .build();

        Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
                .lineToLinearHeading(new Pose2d(-38, -45, Math.toRadians(0))) // Move to get Wobble #2
                .build();

        Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
                .lineToLinearHeading(new Pose2d(7, -45, Math.toRadians(90))) // Move to Square A
                .build();


        //one ring

        Trajectory oneRing1 = drive.trajectoryBuilder(startPose)  // Move to shooting line
                .splineToConstantHeading(new Vector2d(-3, -45), Math.toRadians(0))
                .build();

        Trajectory oneRing2 = drive.trajectoryBuilder(oneRing1.end()) // Move to Square B
                .lineToLinearHeading(new Pose2d(35, -25, Math.toRadians(90)))
                .build();

        Trajectory oneRing3 = drive.trajectoryBuilder(oneRing2.end())  // Move to Wobble 2
                .lineToLinearHeading(new Pose2d(-38, -45, Math.toRadians(0)))
                .build();

        Trajectory oneRing4 = drive.trajectoryBuilder(oneRing3.end()) // Move to Square B
                .lineToLinearHeading(new Pose2d(32, -18, Math.toRadians(90)))
                .build();

        Trajectory oneRing5 = drive.trajectoryBuilder(oneRing4.end()) // Move to white line
                .lineToLinearHeading(new Pose2d(9, -18, Math.toRadians(0)))
                .build();


        //four rings

        Trajectory fourRing1 = drive.trajectoryBuilder(startPose) // Move to shooting line
                .splineToConstantHeading(new Vector2d(-3, -45), Math.toRadians(0))
                .build();

        Trajectory fourRing2 = drive.trajectoryBuilder(fourRing1.end())  // Move to Square C
                .lineToLinearHeading(new Pose2d(60, -52, Math.toRadians(90)))
                .build();

        Trajectory fourRing3 = drive.trajectoryBuilder(fourRing2.end())  // Move to Wobble 2
                .lineToLinearHeading(new Pose2d(-38, -45, Math.toRadians(0)))
                .build();

        Trajectory fourRing4 = drive.trajectoryBuilder(fourRing3.end())  // Return to Square C
                .lineToLinearHeading(new Pose2d(60, -45, Math.toRadians(90)))
                .build();

        Trajectory fourRing5 = drive.trajectoryBuilder(fourRing4.end()) // Move to white line
                .lineToLinearHeading(new Pose2d(9, -18, Math.toRadians(0)))
                .build();


        /*
         *   loop infinitely:
         *       driving to shooting line:
         *           follow trajectory
         *       followTrajectory() <- blocks until completion
         *      followTrajectoryAsync() <- you have to call update() but you can do stuff in between.
         *
         */


        waitForStart();
        if (isStopRequested()) return;

        state = ZERO_RINGS_STATE.INITIALIZING;

        if (rings == 0) {
            while (opModeIsActive()) {
                switch (state) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(zeroRings1);  // Kick off moving to SHOOTING LINE
                        pid.start(targetVelocity);
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
                            wobbleArm1.setPosition(wobbleArmUp);
                            wobbleArm2.setPosition(wobbleArmUp);
                            autonStartTime = System.currentTimeMillis();
                            state = ZERO_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 300) {
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

        if (rings == 1) {
            while (opModeIsActive()) {
                pid.loop();
                switch (state2) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(oneRing1);
                        pid.start(targetVelocity);
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
                            wobbleArm1.setPosition(wobbleArmUp); // MOVE ARM SO IT DOESN'T KNOCK OVER WOBBLE GOAL
                            wobbleArm2.setPosition(wobbleArmUp);
                            state2 = ONE_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 300) {
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

        if (rings == 4) {
            while (opModeIsActive()) {
                pid.loop();
                switch (state3) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        intakeServo.setPosition(intakeServoOpen);
                        drive.followTrajectoryAsync(fourRing1);
                        pid.start(targetVelocity);
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
                            wobbleArm1.setPosition(wobbleArmUp);
                            wobbleArm2.setPosition(wobbleArmUp);
                            state3 = FOUR_RINGS_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        autonDeltaTime = System.currentTimeMillis() - autonStartTime;
                        if (autonDeltaTime > 300) {
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


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

}