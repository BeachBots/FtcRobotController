package org.firstinspires.ftc.teamcode;

import  com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name = "AutonTest")
public class AutonTest extends LinearOpMode
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

    public void strafe(double power, int distance){

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setTargetPosition(distance);
        motorFrontLeft.setTargetPosition(-distance);
        motorBackRight.setTargetPosition(-distance);
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




    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //detect using camera

//half the distance from the wall to the rings
        driveForwardDistance(0.5, 100);
        sleep(100);

//strafe left to line up with power shot
        strafe(0.5, 100);
        sleep(100);

//Move to the launch line to shoot
        driveForwardDistance(0.5,100);
        sleep(100);

//splits off here depending on abc

//strafe to the right to line up with wobble goal zones
        strafe(-0.5, 100);
        sleep(100);

//go forward
        driveForwardDistance(0.5,100);
        sleep(100);

//put down wobble goal

//drive backwards to get other wobble goal
        driveForwardDistance(-0.5, 100);
        sleep(100);

//strafe left to pick up other wobble goal
        strafe(0.5, 100);
        sleep(100);

//drive backwards a little to get other wobble goal
        driveForwardDistance(-0.5, 100);
        sleep(100);

//strafe right a little to pick up other wobble goal
        strafe(-0.5, 100);
        sleep(100);

//pick up wobble goal

//strafe right to line up with zone
        strafe(-0.5, 100);
        sleep(100);

//go forward and drop off the wobble goal
        driveForwardDistance(0.5, 100);
        sleep(100);

//go backwards over the line
        driveForwardDistance(-0.5, 100);
        sleep(100);

//strafe left to line up with rings
        strafe(0.5, 100);
        sleep(100);

//spin 180
        turn(0.5, 180);









    }


}