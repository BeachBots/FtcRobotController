package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ShootPowerTest")
public class ShootPowerTest extends LinearOpMode {

    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo flick;
    private Servo stopper;
    private DcMotor intake;



    @Override
    public void runOpMode() throws InterruptedException {

        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");

        // motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);



        double power = 1;


        flick.setPosition(0.5);
        stopper.setPosition(0.0);
        waitForStart();
        waitForStart();



        while (opModeIsActive()) {

            boolean shootOn = false;

            if (gamepad1.a && shootOn == false){
                shoot1.setPower(-power);
                shoot2.setPower(-power);
                shootOn = true;
            } else if (gamepad1.a && shootOn == true){
                shoot1.setPower(0);
                shoot2.setPower(0);
                shootOn = false;
            }


            boolean intaking = false;

            if (gamepad1.b && intaking == false){
                intake.setPower(1);
                intaking = true;
            } else if (gamepad1.b && intaking == true){
                intake.setPower((0));
                intaking = false;
            }


            boolean output = false;

            if (gamepad1.b && output == false){
                intake.setPower(-1);
                output = true;
            } else if (gamepad1.b && output == true){
                intake.setPower((0));
                output = false;
            }



            if (gamepad1.y){
                flick.setPosition(0);
                sleep(30);
                flick.setPosition(0.5);
            }



            if (gamepad1.dpad_up){
                power = power + 0.1;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_down){
                power = power - 0.1;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_right){
                power = power + 0.01;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }

            if (gamepad1.dpad_left){
                power = power - 0.01;
                telemetry.addData("power = ", power);
                telemetry.update();
                sleep(500);
            }







        }
        idle();
    }
}


