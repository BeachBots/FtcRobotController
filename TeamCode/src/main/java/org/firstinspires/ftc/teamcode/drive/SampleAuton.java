package org.firstinspires.ftc.teamcode.drive;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous (name="SampleAuton", group="LinearOpmode")

public class SampleAuton extends LinearOpMode {

    private Servo flick;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATOQy0T/////AAABmQ/SASRSuUzAugTNY1JUz6NSX/K0IyvPgOTMGlPcjAUyxoQlqULPX1jcW4C4fMzALWwznPmVdS4QFyFERfGevgAJPX1U8c6c1wfPTrhaqwhhoG0SBo/8b6iGaeweb65NN1Xu7PG+LHieN8rr339wsfsGROM9TW3oRp6uYdDFq30aM9tAIadifcbtQq9XUSdUyzF7Owgr8QjIbAw57OYIb6Bwl+7tenxUVSM+pYEkZyVdbCWAWEHAeu10tX7qGXmPEENNiKfnTT/TvGCsjTVK0Xa536mx70V74J/wBadTw32md6QMKjQmWWtCqyYnYZenYws42NGZzXsD/cLU+IbXOIJBaohDU8SaPr7mienrXGgc";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean isRunning = true;

    public enum ZERO_RINGS_STATE{
        INITIALIZING,
        DRIVE_TO_LINE,
        DRIVE_RIGHT,
        DROP_WOBBLE_1,
        DRIVE_TO_WOBBLE_2,
        PICK_UP_WOBBLE_2,
        MOVE_TO_DROP_WOBBLE_2,
        DROP_WOBBLE_2,
        DONE,
    };

    private ZERO_RINGS_STATE state;

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



        while (isStarted() == false){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    if (updatedRecognitions.size() == 0){
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
            }}

        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        flick = hardwareMap.servo.get("flick");

        double setFlickTime = .1;
        double setFlickBetween = 1.0;


        // This identifies the starting position of our robot -- otherwise it default to (0,0) which is the center of the field.
        // We should tune this number, which I'm estimating to be (-63, -30).
            Pose2d startPose = new Pose2d(-63, -30, Math.toRadians(0));
            drive.setPoseEstimate(startPose);


        // This is where we build all the trajectories. We won't call them until the bottom.

        // #1: MOVE UP TO SHOOTING LINE
            Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(5.0, 8.0), Math.toRadians(0))

                //.addTemporalMarker(.5, () -> {
                        // This marker runs .5 seconds into the trajectory. We can fire up the shooter here

                        // Run your action in here!

                //   })

                .build();

        //  #2: MOVE RIGHT WHILE TAKING POWER SHOTS
            Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                 //.splineToConstantHeading(new Vector2d(7.0,-40.0), Math.toRadians(0))
                    .strafeTo(new Vector2d(5.0,-40))

                  // This marker runs .3 seconds into the trajectory. This will be where we trigger shooting loop

                         .addTemporalMarker (.2,()-> {

                         // Run shoot loop here!
                             flick.setPosition (0);
                             flick.setPosition (0.5);
                             flick.setPosition (0);
                             flick.setPosition (0.5);
                             flick.setPosition (0);
                             flick.setPosition (0.5);

                                        })

                 .build();

        //  WOBBLE GOAL DROP ROUTINE OCCURS HERE, BUT IS CALLED BELOW

        //  #3 MOVE TO PICK UP WOBBLE GOAL 2
            Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
                .splineToConstantHeading(new Vector2d(-45,-33), Math.toRadians(0))
                .build();

        //  WOBBLE GOAL PICK UP ROUTINE OCCURS HERE, BUT IS CALLED BELOW

        //  #4 MOVE TO BOX A TO DROP OFF WOBBLE GOAL 2, AND PARK ON WHITE LINE
            Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
                 .splineToConstantHeading(new Vector2d(15, -40), Math.toRadians(0))
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

        if(isStopRequested()) return;

        state = ZERO_RINGS_STATE.INITIALIZING;
        long lastSavedTimeMs = 0;

        while(opModeIsActive()){
            switch(state){
                case INITIALIZING:
                    drive.followTrajectoryAsync(zeroRings1);  // Kick off moving to SHOOTING LINE
                    // pid.start(targetVelocity, targetPower);
                    state = ZERO_RINGS_STATE.DRIVE_TO_LINE;
                    break;
                case DRIVE_TO_LINE:
                    if(drive.isBusy()){ // Still moving to shooting line.
                        drive.update();
                        // pid.update();
                    } else { // change to else if (pid.done()) when we can check it
                        drive.followTrajectoryAsync(zeroRings2);  // MOVE RIGHT WHILE SHOOTING
                        state = ZERO_RINGS_STATE.DRIVE_RIGHT;
                    }
                    break;
                case DRIVE_RIGHT:
                    if(drive.isBusy()){
                        drive.update();
                    } else {
                        lastSavedTimeMs = System.currentTimeMillis(); // prepare for a sleep
                        state = ZERO_RINGS_STATE.DROP_WOBBLE_1;
                    }
                    break;
                case DROP_WOBBLE_1:
                    if(System.currentTimeMillis() - lastSavedTimeMs > 1000) { // Move on if 1000 ms has passed
                        state = ZERO_RINGS_STATE.DRIVE_TO_WOBBLE_2;
                    }
                    break;
                case DRIVE_TO_WOBBLE_2:
                    drive.followTrajectory(zeroRings3);  // MOVE TO PICK UP WOBBLE 2
                    state = ZERO_RINGS_STATE.PICK_UP_WOBBLE_2;
                    break;
                case PICK_UP_WOBBLE_2:
                    sleep (1000); // CALL WOBBLE GOAL PICK UP ROUTINE
                    state = ZERO_RINGS_STATE.MOVE_TO_DROP_WOBBLE_2;
                    break;
                case MOVE_TO_DROP_WOBBLE_2:
                    drive.followTrajectory(zeroRings4);  // MOVE TO DROP OFF WOBBLE 2
                    state = ZERO_RINGS_STATE.DROP_WOBBLE_2;
                    break;
                case DROP_WOBBLE_2:
                    // CALL WOBBLE GOAL DROP ROUTINE
                    state = ZERO_RINGS_STATE.DONE;
                    break;
                case DONE:
                    break;
                default:
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

    private void initTfod(){
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