package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "TeleOP")
public class TeleOpGood extends LinearOpMode {

    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo flick;
    private Servo stopper;
    private DcMotor intake;
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;



    @Override
    public void runOpMode() throws InterruptedException {

        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        intake = hardwareMap.dcMotor.get("intake");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");


        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);



        double power = 1;


        flick.setPosition(0.5);
        stopper.setPosition(0.0);
        waitForStart();
        waitForStart();



        while (opModeIsActive()) {


            float deadZone = (float) 0.9;
            gamepad1.setJoystickDeadzone(deadZone);

            motorFrontRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);
            motorBackRight.setPower(-gamepad1.left_stick_y * 0.8 - gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorFrontLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 + gamepad1.left_stick_x * 0.8);
            motorBackLeft.setPower(-gamepad1.left_stick_y * 0.8 + gamepad1.right_stick_x * 0.8 - gamepad1.left_stick_x * 0.8);






            if (gamepad1.cross){

                shoot1.setPower(-power);
                shoot2.setPower(-power);
            }
            if (gamepad1.circle){

                shoot1.setPower(0);
                shoot2.setPower(0);
            }




            if (gamepad1.right_bumper){
                intake.setPower(1);
            }
            if (gamepad1.left_bumper){
                intake.setPower((0));
            }

/*
            boolean output = false;

            if (gamepad1.x && output == false){
                output = true;

                intake.setPower(-0.5);
                sleep(100);
            } else if (gamepad1.b && output == true){
                output = false;
                intake.setPower((0));
                sleep(100);
            }*/

           /* ///stopper code
            if (gamepad1.y){
                stopper.setPosition(1);
                sleep (100);
                stopper.setPosition(0);
                sleep (100);
            }*/


           if (gamepad1.y){
                flick.setPosition(0);
                sleep(100);
                flick.setPosition(.5);
                sleep(400);
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


