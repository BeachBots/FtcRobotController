package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "WobbleTest")
public class WobbleTest extends LinearOpMode {
    private DcMotor wobbleMotor;
    private Servo wobbleClaw;



    public void motortest(double power, int distance) {
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleMotor.setTargetPosition(-distance);

        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wobbleMotor.setPower(power);

        while (wobbleMotor.isBusy()) {

        }

        wobbleMotor.setPower(0);



    }

    @Override
    public void runOpMode() throws InterruptedException {
        wobbleMotor = hardwareMap.dcMotor.get("wobbleMotor");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");



        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int i = 0;

        if (i==0) {
            wobbleMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
            i = 1;
        }

        double wobbleOpen = .8;
        double wobbleClosed = .6;




        waitForStart();
        waitForStart();


        while (opModeIsActive()) {


        //THIS IS CODE TO TEST MOTOR POSITIONS

            if (gamepad1.a) {
                motortest(.5, 50);
            }

            if (gamepad1.y) {
                motortest(1, 0);
            }

            if (gamepad1.b) {
                motortest(.5, 100);
            }
            if (gamepad1.x) {
                motortest(.5, -50);
            }



        //THIS IS CODE TO TEST SERVO POSITIONS

          /*  if (gamepad1.b) {
                wobbleClaw.setPosition(wobbleOpen);
            }
            if (gamepad1.x){
                wobbleClaw.setPosition(wobbleClosed);
            }
            */


        }
        idle();
    }
}


