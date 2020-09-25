package org.firstinspires.ftc.teamcode;

import  com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "GyroTele")
public class GyroTele extends LinearOpMode
{
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public void driveForwardDistance(double power, int distance){

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(distance);
        motorBackLeft.setTargetPosition(distance);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy()){

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

    public void turn(double power, double angle){


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);


        motorFrontRight.setTargetPosition((int)((((3600/360) * angle))));
        motorBackRight.setTargetPosition((int)((((3600/360) * angle))));
        motorFrontLeft.setTargetPosition((int)((-((3600/360) * angle))));
        motorBackLeft.setTargetPosition((int)((-((3600/360) * angle))));

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy()){

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

    public void turnRight(double power, double angle){


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);


        motorFrontRight.setTargetPosition((int)((-((3590/360) * angle))));
        motorBackRight.setTargetPosition((int)((-((3590/360) * angle))));
        motorFrontLeft.setTargetPosition((int)((((3590/360) * angle))));
        motorBackLeft.setTargetPosition((int)((((3590/360) * angle))));

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        while (motorFrontRight.isBusy() && motorFrontLeft.isBusy() && motorBackRight.isBusy() && motorBackLeft.isBusy()){

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
    public void runOpMode() throws InterruptedException
    {
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);



        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();

        }

        waitForStart();
        waitForStart();

        while (opModeIsActive())
        {

            while (gamepad1.a) {
                driveForwardDistance(1,538);

            }


            motorFrontRight.setPower(-gamepad1.right_stick_y/2);
            motorBackRight.setPower(-gamepad1.right_stick_y/2);
            motorFrontLeft.setPower(-gamepad1.right_stick_y/2);
            motorBackLeft.setPower(-gamepad1.right_stick_y/2);



            /*
            motorFrontRight.setPower(-gamepad1.right_trigger/2);
            motorBackRight.setPower(-gamepad1.right_trigger/2);
            motorFrontLeft.setPower(-gamepad1.right_trigger/2);
            motorBackLeft.setPower(-gamepad1.right_trigger/2);

            motorFrontRight.setPower(gamepad1.left_trigger/2);
            motorBackRight.setPower(gamepad1.left_trigger/2);
            motorFrontLeft.setPower(gamepad1.left_trigger/2);
            motorBackLeft.setPower(gamepad1.left_trigger/2);
            */




            //Turning function

            while (gamepad1.left_stick_x > 0.3 || gamepad1.left_stick_x < -0.3 || gamepad1.left_stick_y > 0.3 || gamepad1.left_stick_y < -0.3){
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double prevAngle = -angles.firstAngle;

                double theta1 = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double angle1 = Math.toDegrees(theta1);
                double angle2 = angle1 - prevAngle;
                //telemetry.addData("prevAngle", prevAngle);
                //telemetry.addData("newAngle", angle1);
                //telemetry.addData("finalAngle", angle2);
                //telemetry.update();

                if (Math.abs(angle2) > 180){
                    double otherAngle = (180 - Math.abs(prevAngle)) + (180 - Math.abs(angle1));
                    //telemetry.addData("otherAngle", otherAngle);
                    //telemetry.update();
                    if (prevAngle > 0) {
                        if (Math.abs(angle2) > otherAngle) {
                            turn(0.5, otherAngle);
                        } else {
                            turn(0.5, angle2);
                        }

                    } else if (prevAngle < 0){
                        if (angle2 > otherAngle) {
                            turnRight(0.5, otherAngle);
                        } else {
                            turn(0.5, angle2);

                        }
                    }
                } else {
                    turn(0.5, angle2);

                }

            }




            while (gamepad1.dpad_right == true)
            {
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(0.5);
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(-0.5);
            }
            while (gamepad1.dpad_left == true)
            {
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(0.5);


            }
            idle();

        }
        stop();
    }


}