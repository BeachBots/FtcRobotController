package org.firstinspires.ftc.teamcode.drive;

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
import org.firstinspires.ftc.teamcode.ShooterStateMachine;

import java.util.List;

@Autonomous(name = "SampleAuton", group = "LinearOpmode")

public class SampleAuton extends LinearOpMode {

    private Servo flick;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo stopper;
    private DcMotor intake;

    private ShooterStateMachine shooter = new ShooterStateMachine();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATOQy0T/////AAABmQ/SASRSuUzAugTNY1JUz6NSX/K0IyvPgOTMGlPcjAUyxoQlqULPX1jcW4C4fMzALWwznPmVdS4QFyFERfGevgAJPX1U8c6c1wfPTrhaqwhhoG0SBo/8b6iGaeweb65NN1Xu7PG+LHieN8rr339wsfsGROM9TW3oRp6uYdDFq30aM9tAIadifcbtQq9XUSdUyzF7Owgr8QjIbAw57OYIb6Bwl+7tenxUVSM+pYEkZyVdbCWAWEHAeu10tX7qGXmPEENNiKfnTT/TvGCsjTVK0Xa536mx70V74J/wBadTw32md6QMKjQmWWtCqyYnYZenYws42NGZzXsD/cLU+IbXOIJBaohDU8SaPr7mienrXGgc";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean isRunning = true;

    public enum ZERO_RINGS_STATE {
        INITIALIZING,
        DRIVE_TO_LINE,
        DRIVE_RIGHT,
        DROP_WOBBLE_1,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        MOVE_TO_DROP_WOBBLE_2,
        DROP_WOBBLE_2,
        DONE,
    }

    public enum ONE_RINGS_STATE {
        INITIALIZING,
        DRIVING_TO_RING_STACK,
        SHOOTING_HIGH_GOALS,
        DRIVING_TO_SQUARE_B,
        DROP_WOBBLE_1,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        MOVE_TO_DROP_WOBBLE_2,
        DROP_WOBBLE_2,
        DRIVE_TO_LINE,
        DONE,
    }

    public enum FOUR_RINGS_STATE {
        INITIALIZING,
        DRIVING_TO_RING_STACK,
        SHOOTING_HIGH_GOALS,
        DRIVING_TO_SQUARE,
        DROP_WOBBLE_1,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        MOVE_TO_DROP_WOBBLE_2,
        DROP_WOBBLE_2,
        DRIVE_TO_LINE,
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
       You can also check if pid.done() before ending a state if it's important the PID has reached
       the correct speed before moving on.
     */


    ;

    private ZERO_RINGS_STATE state;
    private ONE_RINGS_STATE state2;
    private FOUR_RINGS_STATE state3;

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
    private SampleAuton.PID_STATE state1;

    public void start(double inTargetVelocity, double inPower) {
        state1 = SampleAuton.PID_STATE.RUNNING;

        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");
        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");


        shoot1.setDirection(DcMotor.Direction.REVERSE);  // BETTER TO DO THIS HERE THAN SETTING
        shoot2.setDirection(DcMotor.Direction.REVERSE);  // MOTOR POWER TO A NEGATIVE NUMBER LATER
        shooter.init(hardwareMap);

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
            state1 = SampleAuton.PID_STATE.DONE;
        }
    }

    public boolean done(){
        return state1 == SampleAuton.PID_STATE.DONE;
    }

    @Override
    public void runOpMode() {


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

        waitForStart();



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        double setFlickTime = .1;
        double setFlickBetween = 1.0;



        //zero rings

        // This identifies the starting position of our robot -- otherwise it default to (0,0) which is the center of the field.
        // We should tune this number, which I'm estimating to be (-63, -30).
        Pose2d startPose = new Pose2d(-63, -30, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        // This is where we build all the trajectories. We won't call them until the bottom.

        // #1: MOVE UP TO SHOOTING LINE
        Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-35, -37), Math.toRadians(0))

                //.addTemporalMarker(.5, () -> {
                // This marker runs .5 seconds into the trajectory. We can fire up the shooter here

                // Run your action in here!

                //   })

                .build();

        //  #2: MOVE RIGHT WHILE TAKING POWER SHOTS
        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                //.splineToConstantHeading(new Vector2d(7.0,-40.0), Math.toRadians(0))
                .strafeTo(new Vector2d(10, -37))

                // This marker runs .3 seconds into the trajectory. This will be where we trigger shooting loop

                .addTemporalMarker(.2, () -> {

                    // Run shoot loop here!
                    flick.setPosition(0);
                    flick.setPosition(0.5);
                    flick.setPosition(0);
                    flick.setPosition(0.5);
                    flick.setPosition(0);
                    flick.setPosition(0.5);

                })

                .build();

        //  WOBBLE GOAL DROP ROUTINE OCCURS HERE, BUT IS CALLED BELOW

        //  #3 MOVE TO PICK UP WOBBLE GOAL 2
        Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
                .splineToConstantHeading(new Vector2d(-50, -37), Math.toRadians(0))
                .build();

        //  WOBBLE GOAL PICK UP ROUTINE OCCURS HERE, BUT IS CALLED BELOW

        //  #4 MOVE TO BOX A TO DROP OFF WOBBLE GOAL 2, AND PARK ON WHITE LINE
        Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
                .splineToConstantHeading(new Vector2d(10, -37), Math.toRadians(0))
                .build();



        //one ring

        Trajectory oneRing1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-35, -37), Math.toRadians(0))
                .build();

        Trajectory oneRing2 = drive.trajectoryBuilder(oneRing1.end())
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(0))
                .build();

        Trajectory oneRing3 = drive.trajectoryBuilder(oneRing2.end())
                .splineToConstantHeading(new Vector2d(-50, -20), Math.toRadians(0))
                .build();

        Trajectory oneRing4 = drive.trajectoryBuilder(oneRing3.end())
                .splineToConstantHeading(new Vector2d(-50, -37), Math.toRadians(0))
                .build();

        Trajectory oneRing5 = drive.trajectoryBuilder(oneRing4.end())
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(0))
                .build();

        Trajectory oneRing6 = drive.trajectoryBuilder(oneRing5.end())
                .splineToConstantHeading(new Vector2d(12, -20), Math.toRadians(0))
                .build();



        //four rings

        Trajectory fourRing1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-35, -37), Math.toRadians(0))
                .build();

        Trajectory fourRing2 = drive.trajectoryBuilder(fourRing1.end())
                .splineToConstantHeading(new Vector2d(60, -37), Math.toRadians(0))
                .build();

        Trajectory fourRing3 = drive.trajectoryBuilder(fourRing2.end())
                .splineToConstantHeading(new Vector2d(-50, -37), Math.toRadians(0))
                .build();

        Trajectory fourRing4 = drive.trajectoryBuilder(fourRing3.end())
                .splineToConstantHeading(new Vector2d(60, -37), Math.toRadians(0))
                .build();

        Trajectory fourRing5 = drive.trajectoryBuilder(fourRing4.end())
                .splineToConstantHeading(new Vector2d(12, -37), Math.toRadians(0))
                .build();




        //  WOBBLE GOAL DROP OFF ROUTINE OCCURS HERE, BUT IS CALLED BELOW


        // This is where we call all the trajectories. For now I've added sleeps in between themCto simulate where actions go.

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
        long lastSavedTimeMs = 0;

        if (rings == 0){
        while (opModeIsActive()) {
            switch (state) {
                case INITIALIZING:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    drive.followTrajectoryAsync(zeroRings1);  // Kick off moving to SHOOTING LINE
                     start(targetVelocity, targetPower);
                    state = ZERO_RINGS_STATE.DRIVE_TO_LINE;
                    break;
                case DRIVE_TO_LINE:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    if (drive.isBusy()) { // Still moving to shooting line.
                        drive.update();
                       update();
                    } else if (done()) { // change to else if (pid.done()) when we can check it
                        shooter.shoot(3);
                        state = ZERO_RINGS_STATE.DRIVE_RIGHT;
                    }
                    break;
                case DRIVE_RIGHT:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE ) {
                        shooter.loop();
                    } else {
                        lastSavedTimeMs = System.currentTimeMillis(); // prepare for a sleep
                        state = ZERO_RINGS_STATE.DROP_WOBBLE_1;
                    }
                    break;
                case DROP_WOBBLE_1:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    if (System.currentTimeMillis() - lastSavedTimeMs > 1000) { // Move on if 1000 ms has passed

                        state = ZERO_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                    }
                    break;
                case DRIVE_TO_WOBBLE_2:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    drive.followTrajectory(zeroRings3);  // MOVE TO PICK UP WOBBLE 2
                    state = ZERO_RINGS_STATE.PICK_UP_WOBBLE_2;
                    break;
                case PICK_UP_WOBBLE_2:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    sleep(1000); // CALL WOBBLE GOAL PICK UP ROUTINE
                    state = ZERO_RINGS_STATE.MOVE_TO_DROP_WOBBLE_2;
                    break;
                case MOVE_TO_DROP_WOBBLE_2:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    drive.followTrajectory(zeroRings4);  // MOVE TO DROP OFF WOBBLE 2
                    state = ZERO_RINGS_STATE.DROP_WOBBLE_2;
                    break;
                case DROP_WOBBLE_2:
                    telemetry.addData("state = ", state);
                    telemetry.update();
                    // CALL WOBBLE GOAL DROP ROUTINE
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
        if (rings == 1){
            while (opModeIsActive()) {
                switch (state2) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectoryAsync(oneRing1);
                        update();
                        state2 = ONE_RINGS_STATE.DRIVING_TO_RING_STACK;
                        break;
                    case DRIVING_TO_RING_STACK:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (drive.isBusy()) { // Still moving to shooting line.
                            drive.update();
                            update();
                        } else if (done()) { // change to else if (pid.done()) when we can check it
                            shooter.shoot(3);
                            state2 = ONE_RINGS_STATE.SHOOTING_HIGH_GOALS;
                        }
                        break;
                    case SHOOTING_HIGH_GOALS:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        if (shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE ) {
                            shooter.loop();
                        } else {
                            lastSavedTimeMs = System.currentTimeMillis(); // prepare for a sleep
                            state2 = ONE_RINGS_STATE.DRIVING_TO_SQUARE_B;
                        }
                        break;
                    case DRIVING_TO_SQUARE_B:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing2);
                        state2 = ONE_RINGS_STATE.DROP_WOBBLE_1;
                        break;
                    case DROP_WOBBLE_1:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        //wobble goal stuff
                        state2 = ONE_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                        break;
                    case DRIVE_TO_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing3);
                        drive.followTrajectory(oneRing4);
                        state2 = ONE_RINGS_STATE.PICK_UP_WOBBLE_2;
                        break;
                    case PICK_UP_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        //wobble goal stuff
                        state2 = ONE_RINGS_STATE.MOVE_TO_DROP_WOBBLE_2;
                        break;
                    case MOVE_TO_DROP_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing5);
                        state2 = ONE_RINGS_STATE.DROP_WOBBLE_2;
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        //wobble goal stuff
                        state2 = ONE_RINGS_STATE.DRIVE_TO_LINE;
                        break;
                    case DRIVE_TO_LINE:
                        telemetry.addData("state = ", state2);
                        telemetry.update();
                        drive.followTrajectory(oneRing6);
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

        if (rings == 4){
            while (opModeIsActive()) {
                switch (state3) {
                    case INITIALIZING:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectoryAsync(fourRing1);
                        update();
                        state3 = FOUR_RINGS_STATE.DRIVING_TO_RING_STACK;
                        break;
                    case DRIVING_TO_RING_STACK:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        if (drive.isBusy()) { // Still moving to shooting line.
                            drive.update();
                            update();
                        } else if (done()) { // change to else if (pid.done()) when we can check it
                            shooter.shoot(3);
                            state3 = FOUR_RINGS_STATE.SHOOTING_HIGH_GOALS;
                        }
                        break;
                    case SHOOTING_HIGH_GOALS:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        if ( shooter.shooterState != ShooterStateMachine.ShooterState.SHOOTER_IDLE) {
                            shooter.loop();
                        } else {
                            lastSavedTimeMs = System.currentTimeMillis(); // prepare for a sleep
                            state3 = FOUR_RINGS_STATE.DRIVING_TO_SQUARE;
                        }
                        break;
                    case DRIVING_TO_SQUARE:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectory(fourRing2);
                        state3 = FOUR_RINGS_STATE.DROP_WOBBLE_1;
                        break;
                    case DROP_WOBBLE_1:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        //wobble goal stuff
                        state3 = FOUR_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                        break;
                    case DRIVE_TO_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectory(fourRing3);
                        state3 = FOUR_RINGS_STATE.PICK_UP_WOBBLE_2;
                        break;
                    case PICK_UP_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        //wobble goal stuff
                        state3 = FOUR_RINGS_STATE.MOVE_TO_DROP_WOBBLE_2;
                        break;
                    case MOVE_TO_DROP_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        drive.followTrajectory(fourRing4);
                        state3 = FOUR_RINGS_STATE.DROP_WOBBLE_2;
                        break;
                    case DROP_WOBBLE_2:
                        telemetry.addData("state = ", state3);
                        telemetry.update();
                        //wobble goal stuff
                        state3 = FOUR_RINGS_STATE.DRIVE_TO_LINE;
                        break;
                    case DRIVE_TO_LINE:
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


// SLOP CODE:

//FLICK ONE
//                .addTemporalMarker(setFlickTime, () -> {
//                    flick.setPosition(0.5);
//                })
//
//                .addTemporalMarker(setFlickTime + setFlickBetween, () -> {
//                    flick.setPosition(0.0);
//                })
//
//               // .strafeTo(new Vector2d(-3, -10))
//
////            //FLICK TWO
////                .addTemporalMarker(setFlickTime*2 + setFlickBetween, () -> {
////                    flick.setPosition(0.5);
////                })
////                .addTemporalMarker(2*setFlickTime + 2*setFlickBetween, () -> {
////                    flick.setPosition(0.0);
////                })
////
////                .strafeTo(new Vector2d(-3,-15))
////
////            //FLICK THREE
////                .addTemporalMarker(3*setFlickTime + 2*setFlickBetween, () -> {
////                    flick.setPosition(0.5);
////                })
////                .addTemporalMarker(3*setFlickTime + 2*setFlickBetween, () -> {
////                    flick.setPosition(0.0);
////                })
//                .build();

//  HERE'S A VERSION USING DISPLACEMENT MARKERS INSTEAD OF TEMPORAL ONES
        /*      //POWER SHOT 1//
                    .addDisplacementMarker(() -> {
                        // Run power shot action in here!
                        sleep(500);
                    })
                .strafeRight(7.0)
                //POWER SHOT 2//
                    .addDisplacementMarker(() -> {
                        // Run power shot action in here!
                        sleep(500);
                    })
                .strafeRight(7.0)
                //POWER SHOT 3//
                    .addDisplacementMarker(() -> {
                        // Run power shot action in here!
                        sleep(500);
                    })
                .build();*/

// HERE'S A VERSION USING ANGLES INSTEAD OF STRAFING. Each turn iS 2.5 degrees - this is an estimate

           /*    Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())

             //FLICK ONE
                .addTemporalMarker(setFlickTime, () -> {
                    flick.setPosition(0.0);
                 })
                .addTemporalMarker(setFlickTime + setFlickBetween, () -> {
                    flick.setPosition(0.0);
                })

  //              .lineToLinearHeading(new Pose2d(-3.0,-5.0), Math.toRadians(3)))
                .lineToLinearHeading(new Pose2d(-3.0,-5.0), Math.toRadians(30))

             //FLICK TWO
                .addTemporalMarker(setFlickTime*2 + setFlickBetween, () -> {
                    flick.setPosition(0.5);
                })
                .addTemporalMarker(2*setFlickTime + 2*setFlickBetween, () -> {
                    flick.setPosition(0.0);
                })

//                .lineToLinearHeading(new Pose2d(-3.0,-5.0), Math.toRadians(0.0)))
                .lineToLinearHeading(new Pose2d(-3.0,-5.0), Math.toRadians(0))
             //FLICK THREE
                .addTemporalMarker(3*setFlickTime + 2*setFlickBetween, () -> {
                    flick.setPosition(0.5);
                })
                .addTemporalMarker(3*setFlickTime + 2*setFlickBetween, () -> {
                    flick.setPosition(0.0);
                })
                .build();
*/

      /*     OLD VERSION

    // #2: MOVE  POWER SHOT
    Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
            .lineTo(new Vector2d(-3.0,-10.0))
            .build();

    // #3: MOVE AFTER SECOND POWER SHOT
    Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
            .lineTo(new Vector2d(-3.0,-15.0))
            .build();

    // #4 MOVE TO BOX A TO DROP OFF WOBBLE GOAL 1
    Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
            .strafeTo(new Vector2d(1.0, -40.0))
            //WOBBLE GOAL DROP ROUTINE OCCURS HERE, BUT IS CALLED BELOW
            .build();

    // #5 MOVE TO PICK UP WOBBLE GOAL 2
    Trajectory zeroRings5 = drive.trajectoryBuilder(zeroRings4.end())
            .splineTo(new Vector2d(-45,-33), Math.toRadians(0))
            //WOBBLE GOAL PICK UP ROUTINE OCCURS HERE, BUT IS CALLED BELOW
            .build();

    // #6 MOVE TO BOX A TO DROP OFF WOBBLE GOAL 2, AND PARK ON WHITE LINE
    Trajectory zeroRings6 = drive.trajectoryBuilder(zeroRings5.end())
            .splineTo(new Vector2d(5, -40), Math.toRadians(0))
            //WOBBLE GOAL DROP OFF ROUTINE OCCURS HERE, BUT IS CALLED BELOW
            .build();*/