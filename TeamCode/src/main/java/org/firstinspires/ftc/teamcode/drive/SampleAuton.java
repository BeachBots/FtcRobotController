package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous (name="SampleAuton", group="LinearOpmode")

public class SampleAuton extends LinearOpMode {

    private Servo flick;

    @Override
    public void runOpMode() {
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

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(zeroRings1);  // MOVE TO SHOOTING LINE
        drive.followTrajectory(zeroRings2);  // MOVE RIGHT WHILE SHOOTING
        sleep (1000); // CALL WOBBLE GOAL DROP ROUTINE
        drive.followTrajectory(zeroRings3);  // MOVE TO PICK UP WOBBLE 2
        sleep (1000); // CALL WOBBLE GOAL PICK UP ROUTINE
        drive.followTrajectory(zeroRings4);  // MOVE TO DROP OFF WOBBLE 2
        // CALL WOBBLE GOAL DROP ROUTINE

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