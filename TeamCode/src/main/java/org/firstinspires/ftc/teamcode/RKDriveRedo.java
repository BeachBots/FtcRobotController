package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RKdrive redo")
public class RKDriveRedo extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo flick;
    private Servo stopper;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        shoot1 = hardwareMap.dcMotor.get("shoot1");
        shoot2 = hardwareMap.dcMotor.get("shoot2");
        flick = hardwareMap.servo.get("flick");
        stopper = hardwareMap.servo.get("stopper");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double left = 0;
        double right = 0;

        flick.setPosition(0.0);
        stopper.setPosition(0.0);

        waitForStart();
        waitForStart();

        while (opModeIsActive()) {




            double s = gamepad1.left_stick_y;
            double t = gamepad1.right_stick_x;




            motorFrontRight.setPower(-gamepad1.left_stick_y/2 + gamepad1.right_stick_x/2 - gamepad1.left_stick_x/2);
            motorBackRight.setPower(-gamepad1.left_stick_y/2 + gamepad1.right_stick_x/2 + gamepad1.left_stick_x/2);
            motorFrontLeft.setPower(-gamepad1.left_stick_y/2 - gamepad1.right_stick_x/2 + gamepad1.left_stick_x/2);
            motorBackLeft.setPower(-gamepad1.left_stick_y/2 - gamepad1.right_stick_x/2 - gamepad1.left_stick_x/2);

            if (gamepad1.a){
                shoot1.setPower(1);
                shoot2.setPower(1);
                sleep(500);
                stopper.setPosition(0.5);
                sleep(100);
                flick.setPosition(0.5);
                flick.setPosition(0.0);
                sleep(500);
                flick.setPosition(0.5);
                flick.setPosition(0.0);
                sleep(500);
                flick.setPosition(0.5);
                flick.setPosition(0.0);
                sleep(500);
                stopper.setPosition(0.0);
                shoot1.setPower(0.3);
                shoot2.setPower(0.3);
            }




        }
        idle();
    }
}


