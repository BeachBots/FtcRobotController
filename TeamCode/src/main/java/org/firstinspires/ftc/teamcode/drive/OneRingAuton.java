package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.provider.Settings;

import  com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class OneRingAuton {




public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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


//The distances, powers, and sleeps are all estimates. This is a shell that will be tuned in person.

        driveForwardDistance(power0.5, 100);
        sleep(100);

        shoot1.setPower(-1);
        shoot2.setPower(-1);

//(Here I copied the code from the tele op for shooting, so this might not work in auton. I couldn't find the auton shooting code.)

//Shoot all three loaded rings into high goal

        driveForwardDistance(power0.5, 100);
        sleep (100)

//Here we're driving forward so that we can intake the ring more easily

//Intake the one ring on the ground
//Sleep command for the intake

        shoot1.setPower(-1);
        shoot2.setPower(-1);
        sleep(100);

//This is shooting the ring into the high goal and then turning off the shooter

        strafe(0.5, 100);
        sleep(100);

//Here we strafe to the right to allign with the drop zone. We are strafing to the right.

        driveForwardDistance(power0.5, 100);
        sleep(100);

//This command is to get the robot into zone B

//Here we would release the wobble goal into the zone

        driveForwardDistance(power0.5), -100);
        sleep(100);

//This is supposed to let the robot drive backwards so that it can strafe and pick up the wobble. I'm not sure if I did it right.

        strafe(0.5, -100);
        sleep(100);

//Here we are strafing to pick up the wobble goal. We are strafing left.

//Pick up wobble goal

        strafe(0.5, 100);
        sleep(100);

//We are following the exact same path as we did to drop off the first wobble. We are strafing right.

        driveForwardDistance(power0.5, 100);
        sleep(100);

//Release wobble goal

        driveForwardDistance(power0.5, -100);
        sleep(100);

//This is meant to put us over the launch line. We are moving backwards.

    }}}}