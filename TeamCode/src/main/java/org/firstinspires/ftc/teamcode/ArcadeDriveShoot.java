package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Arcade")
public class ArcadeDriveShoot extends LinearOpMode {

    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo flick;
    private Servo stopper;



    @Override
    public void runOpMode() throws InterruptedException {

         shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");

        // motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);






        flick.setPosition(0.5);
        stopper.setPosition(0.0);
        waitForStart();
        waitForStart();



        while (opModeIsActive()) {

            //shoot1.setPower(-0.1);
            //shoot2.setPower(-0.1);

            if (gamepad1.b == true){
                shoot1.setPower(0);
                shoot2.setPower(0);
            }


            if (gamepad1.x){
                flick.setPosition(0);

            }
            if (gamepad1.y){
                flick.setPosition(0.5);
            }

            if (gamepad1.a){
                shoot1.setPower(-1);
                shoot2.setPower(-1);
                //sleep(500);
                //stopper.setPosition(0.5);
                //sleep(100);
                //flick.setPosition(0);
                //flick.setPosition(0.5);
                //sleep(1000);
               // flick.setPosition(0.5);
               // flick.setPosition(0.0);
                //sleep(500);
                //flick.setPosition(0.5);
                //flick.setPosition(0.0);
                //sleep(500);
                //stopper.setPosition(0.0);
                //shoot1.setPower(0);
                //shoot2.setPower(0);
            }






        }
        idle();
    }
}


