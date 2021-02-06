package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.provider.Settings;

import  com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
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

            // Run shoot loop here!
            flick.setPosition (0);
            flick.setPosition (0.5);
            flick.setPosition (0);
            flick.setPosition (0.5);
            flick.setPosition (0);
            flick.setPosition (0.5);

        //  #3: INTAKE REMAINING RING


        //  #4  SHOOT REMAINING RING


        //  #5  GO TO DROP ZONE B

        Trajectory zeroRings2 = drive.trajectoryBuilder(zeroRings1.end())
                .splineToConstantHeading(new Vector2d(30, -37), Math.toRadians(0))

                .build();

        //  #6  DROP WOBBLE GOAL


        // #7  PICK UP SECOND WOBBLE GOAL

        Trajectory zeroRings3 = drive.trajectoryBuilder(zeroRings2.end())
                .lineToConstantHeading(new Vector2d(-50, -47))

                .build();

        //  #8  DROP OFF SECOND WOBBLE GOAL

        Trajectory zeroRings4 = drive.trajectoryBuilder(zeroRings3.end())
                .splineToConstantHeading(new Vector2d(30, -35), Math.toRadians(0))

                .build();

        Trajectory zeroRings5 = drive.trajectoryBuilder(zeroRings4.end())
                .lineToConstantHeading(new Vector2d(10, -35))

                .build();


    }}
