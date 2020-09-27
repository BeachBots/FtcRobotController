package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RKdrive redo")
public class RKDriveRedo extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

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

        double left = 0;
        double right = 0;

        waitForStart();
        waitForStart();

        while (opModeIsActive()) {


            motorFrontRight.setPower(right);
            motorBackRight.setPower(right);
            motorFrontLeft.setPower(left);
            motorBackLeft.setPower(left);


            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2){
                while (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    left = left - gamepad1.left_stick_y;
                    right = right - gamepad1.left_stick_y;
                }} else{
                left = 0;
                right = 0;
            }

            if (gamepad1.right_stick_x > 0.2){
                while (gamepad1.right_stick_x > 0.2) {
                    right = right + gamepad1.left_stick_y;
                    left = left - gamepad1.left_stick_y;
                }} else{
                left = 0;
                right = 0;
            }

            if (gamepad1.right_stick_x < -0.2) {
                while (gamepad1.right_stick_x < -0.2) {
                    right = right + gamepad1.left_stick_y;
                    left = left - gamepad1.left_stick_y;
                } } else {
                left = 0;
                right = 0;
            }

            if (gamepad1.left_stick_x > 0.2){
                while (gamepad1.left_stick_x > 0.2) {
                    motorFrontRight.setPower(-gamepad1.left_stick_x/2);
                    motorBackRight.setPower(gamepad1.left_stick_x/2);
                    motorFrontLeft.setPower(gamepad1.left_stick_x/2);
                    motorBackLeft.setPower(-gamepad1.left_stick_x/2);
                }} else {
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }
            if (gamepad1.left_stick_x < -0.2){
                while (gamepad1.left_stick_x < -0.2) {
                    motorFrontRight.setPower(-gamepad1.left_stick_x/2);
                    motorBackRight.setPower(gamepad1.left_stick_x/2);
                    motorFrontLeft.setPower(gamepad1.left_stick_x/2);
                    motorBackLeft.setPower(-gamepad1.left_stick_x/2);
                }} else{
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }

        }
        idle();
    }
}


