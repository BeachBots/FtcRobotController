package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "distance test")
public class driveTest extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;


    public void driveForwardDistance(double power, int distance) {

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);



        motorFrontRight.setTargetPosition(-distance);
        motorFrontLeft.setTargetPosition(-distance);
        motorBackRight.setTargetPosition(-distance);
        motorBackLeft.setTargetPosition(-distance);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy()) {

        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void runOpMode() throws InterruptedException {
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





        waitForStart();
        waitForStart();

        int distance = 0;

        while (opModeIsActive()) {





            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            float deadZone = (float) 0.2;
            gamepad1.setJoystickDeadzone(deadZone);

            if (gamepad1.right_bumper){
                distance = distance + 1000;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }

            if (gamepad1.left_bumper){
                distance = distance - 1000;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }


          if (gamepad1.a){
              distance = distance + 100;
              telemetry.addData("Ticks = ", distance);
              telemetry.update();
              sleep(200);
          }

            if (gamepad1.b){
                distance = distance - 100;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }

            if (gamepad1.dpad_up){
                distance = distance + 10;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }

            if (gamepad1.dpad_down){
                distance = distance - 10;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }

            if (gamepad1.dpad_right){
                distance = distance + 1;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }

            if (gamepad1.dpad_left){
                distance = distance - 1;
                telemetry.addData("Ticks = ", distance);
                telemetry.update();
                sleep(200);
            }

            if  (gamepad1.y){
                driveForwardDistance(0.5, distance);
            }







        }
        idle();
    }
}


