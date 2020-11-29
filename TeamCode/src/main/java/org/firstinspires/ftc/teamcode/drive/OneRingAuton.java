package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous (name="OneRingAuton", group="LinearOpmode")

public class OneRingAuton extends LinearOpMode {

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

        // #1: MOVE UP TO RING STACK
        Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-37, -37), Math.toRadians(0))

                .build();

        //  #2: SHOOT THREE HIGH GOALS HERE


        //  #3: INTAKE REMAINING RING


        //  #4  SHOOT REMAINING RING


        //  #5  DROP OFF FIRST WOBBLE GOAL

        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                .splineToConstantHeading(new Vector2d(30, -37), Math.to

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