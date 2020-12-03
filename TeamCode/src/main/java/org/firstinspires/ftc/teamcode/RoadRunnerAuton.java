package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "RoadRunnerAuton", group = "LinearOpode")

public class RoadRunnerAuton extends LinearOpMode {

    private Servo flick;
    private Servo stopper;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        flick = hardwareMap.servo.get("flick");
        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");

        // This identifies the starting position of our robot -- otherwise it default to (0,0) which is the center of the field.
        // We should tune this number, which I'm estimating to be (-63, -30).
        Pose2d startPose = new Pose2d(-63, -30, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        // This is where we build all the trajectories. We won't call them until the bottom.

        // #1: MOVE UP TO SHOOTING LINE
        Trajectory zeroRings1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(0))
                .build();

        // SHOOT THREE HIGH GOALS ROUTINE OCCURS HERE, BUT IS CALLED BELOW

        // #2: MOVE TO DROP OFF WOBBLE GOAL 1
        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                .splineToConstantHeading(new Vector2d(12.0, -40.0), Math.toRadians(0))
                .build();

        // MOVE TO PICK UP WOBBLE GOAL 2
        Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
                .splineToConstantHeading(new Vector2d(-50,-36), Math.toRadians(0))
                .build();

        //  WOBBLE GOAL PICK UP ROUTINE OCCURS HERE, BUT IS CALLED BELOW

        //  #4 MOVE TO BOX A TO DROP OFF WOBBLE GOAL 2, AND PARK ON WHITE LINE
        Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
                .splineToConstantHeading(new Vector2d(8, -40), Math.toRadians(0))
                .build();

        //  WOBBLE GOAL DROP OFF ROUTINE OCCURS HERE, BUT IS CALLED BELOW


        // This is where we call all the trajectories. For now I've added sleeps in between themCto simulate where actions go.

        waitForStart();

        if(isStopRequested()) return;

        shoot1.setPower(-.70);  // ADJUST AS NECESSARY
        shoot2.setPower(-.70);  // MAKE SURE IT'S NEGATIVE
        flick.setPosition(0.5);
        sleep(1000);

        drive.followTrajectory(zeroRings1);  // MOVE TO SHOOTING LINE

        //SHOOTING ROUTINE
        flick.setPosition(0);
        sleep (100);
        flick.setPosition(0.5);
        sleep (100);
        flick.setPosition(0);
        sleep (100);
        flick.setPosition(0.5);
        sleep (100);
        flick.setPosition(0);
        sleep (100);
        flick.setPosition(0.5);
        shoot1.setPower (0);
        shoot2.setPower (0);

        drive.followTrajectory(zeroRings2);  // MOVE RIGHT TO DROP OFF WOBBLE 1

        sleep (1000); // CALL WOBBLE GOAL DROP ROUTINE

        drive.followTrajectory(zeroRings3);  // MOVE TO PICK UP WOBBLE 2

        sleep (1000); // CALL WOBBLE GOAL PICK UP ROUTINE

        drive.followTrajectory(zeroRings4);  // MOVE TO DROP OFF WOBBLE 2

        sleep (1000); // CALL WOBBLE GOAL DROP ROUTINE

    }
}
