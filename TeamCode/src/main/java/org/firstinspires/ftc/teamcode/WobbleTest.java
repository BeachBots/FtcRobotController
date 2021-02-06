package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "WobbleTest")
public class WobbleTest extends LinearOpMode {
    private Servo wobbleArm1;
    private Servo wobbleArm2;
    private Servo wobbleClaw;


    @Override
    public void runOpMode() throws InterruptedException {
        wobbleArm1 = hardwareMap.servo.get("wobbleArm1");
        wobbleArm2 = hardwareMap.servo.get("wobbleArm2");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");

        double wobbleClawOpen = .50;
        double wobbleClawClosed = .15;
        double wobbleArmStowed = .14;
        double wobbleArmExtended = .68;
        double wobbleArmUp = .32;
        boolean output = false;

        waitForStart();


        while (opModeIsActive()) {

            /*if (gamepad1.a) {
                wobbleArm1.setPosition(0);
                wobbleArm2.setPosition(0);
            }*/

            if (gamepad1.y) {
                wobbleArm1.setPosition(wobbleArmStowed);
                wobbleArm2.setPosition(wobbleArmStowed);
            }

            if (gamepad1.b) {
                wobbleArm1.setPosition(wobbleArmExtended);
                wobbleArm2.setPosition(wobbleArmExtended);
            }

            if (gamepad1.x) {
                wobbleArm1.setPosition(wobbleArmUp);
                wobbleArm2.setPosition(wobbleArmUp);
            }

            if (gamepad1.back) {
                //output = !output;
                //wobbleClaw.setPosition(output ? wobbleClawClosed : wobbleClawOpen);
                wobbleClaw.setPosition(wobbleClawClosed);
            }

            if (gamepad1.start) {
                wobbleClaw.setPosition(wobbleClawOpen);
            }
        }
        idle();
    }
}


