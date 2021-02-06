package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous (name="PowerShotTest", group="LinearOpmode")

public class PowerShotTest extends LinearOpMode {

    private Servo flick;
    private Servo stopper;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");


        // This identifies the starting position of our robot. To make things easier for testing, make it 0,0.
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        // This is where we build all the trajectories. We won't call them until the bottom.



        // VERSION 1: STRAFE RIGHT IN CONTINUOUS LINE WHILE SHOOTING

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(0.0, -24.0))
                .addTemporalMarker(2.0, () -> {
                    // Fires at 2 seconds
                    flick.setPosition(0.0);
                })
                .addTemporalMarker(2.2, ()-> {
                    flick.setPosition(0.5);
                })
               /* // Retracts flicker after 2.1 seconds
                    flick.setPosition(0.5);
                })

                .addSpatialMarker(new Vector2d(0,-12),() -> {
                    // Fires at 12 inches (7 inches from shot 1)
                    flick.setPosition(0.0);
                })

                .addSpatialMarker(new Vector2d(0,-15),() -> {
                    // Retracts flicker after +3 inches
                    flick.setPosition(0.5);
                })

                .addSpatialMarker(new Vector2d(0,-19),() -> {
                    // Fires at 19 inches (7 inches from shot 2)
                    flick.setPosition(0.0);
                })

                .addSpatialMarker(new Vector2d(0,-20),() -> {
                    // Retracts flicker after +1 inches
                    flick.setPosition(0.5);
                })*/

                .build();


        waitForStart();

        if(isStopRequested()) return;
        while (opModeIsActive()) {

            shoot1.setPower(-0.58);     // Power up motors
            shoot2.setPower(-0.58);
            sleep (2000);
            drive.followTrajectory(trajectory1);
            shoot1.setPower(0);         // Power down motors
            shoot2.setPower(0);


        }
    }


/*

        // VERSION 2:  SHOOT, MOVE, SHOOT

        Trajectory powerShot1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(0.0, -8.0))
                .build();

        Trajectory powerShot2 = drive.trajectoryBuilder(powerShot1.end())
                .lineToConstantHeading(new Vector2d(0.0, -15.0))
                .build();

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            flick.setPosition(0.5);
            //FIRE UP MOTORS
            shoot1.setPower(-0.58);
            shoot2.setPower(-0.58);
            sleep(1500);

            //TAKE SHOT 1
            flick.setPosition(0);
            sleep(100);
            flick.setPosition(0.5);

            //MOVE TO POSITION 2
            drive.followTrajectory(powerShot1);

            //TAKE SHOT 2
            flick.setPosition(0);
            sleep(100);
            flick.setPosition(0.5);

            //MOVE TO POSITION 3
            drive.followTrajectory(powerShot2);

            //TAKE SHOT 3
            flick.setPosition(0);
            sleep(100);
            flick.setPosition(0.5);

            //TURN OFF MOTORS
            shoot1.setPower(0.0);
            shoot2.setPower(0.0);

            }
        }

*/


/*


        // VERSION 3: ANGLE, SHOOT, ANGLE, SHOOT, ANGLE, SHOOT

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //FIRE UP MOTORS
            flick.setPosition(0.5);
            shoot1.setPower(-0.58);
            shoot2.setPower(-0.58);
            sleep (1000);

            //TURN FOR SHOT 1
            drive.turn(Math.toRadians(7.0));

            //TAKE SHOT 1
            flick.setPosition(0);
           // sleep(100);
            flick.setPosition(0.5);

            //TURN FOR SHOT 2
            drive.turn(Math.toRadians(0.0));

            //TAKE SHOT 2
            flick.setPosition(0);
           // sleep(100);
            flick.setPosition(0.5);

            //TURN FOR SHOT 3
            drive.turn(Math.toRadians(-14.0));;

            //TAKE SHOT 3
            flick.setPosition(0);
         //   sleep(100);
            flick.setPosition(0.5);

            //TURN OFF MOTORS
            shoot1.setPower(0.0);
            shoot2.setPower(0.0);

        }
     }


*/



}



