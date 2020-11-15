package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

//test from Jon


@TeleOp(name = "RKdrive")
public class RKDrive extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    OpenGLMatrix lastLocation = null;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    VuforiaLocalizer vuforia;
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraName = webcamName;

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2){
                while (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2) {
                    motorFrontRight.setPower(-gamepad1.left_stick_y / 2);
                    motorBackRight.setPower(-gamepad1.left_stick_y / 2);
                    motorFrontLeft.setPower(-gamepad1.left_stick_y / 2);
                    motorBackLeft.setPower(-gamepad1.left_stick_y / 2);
                }} else{
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }

            if (gamepad1.right_stick_x > 0.2){
                while (gamepad1.right_stick_x > 0.2) {
                    motorFrontRight.setPower(gamepad1.right_stick_x / 2);
                    motorBackRight.setPower(gamepad1.right_stick_x / 2);
                    motorFrontLeft.setPower(-gamepad1.right_stick_x / 2);
                    motorBackLeft.setPower(-gamepad1.right_stick_x / 2);
                }} else{
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }

            if (gamepad1.right_stick_x < -0.2) {
                while (gamepad1.right_stick_x < -0.2) {
                    motorFrontRight.setPower(gamepad1.right_stick_x / 2);
                    motorBackRight.setPower(gamepad1.right_stick_x / 2);
                    motorFrontLeft.setPower(-gamepad1.right_stick_x / 2);
                    motorBackLeft.setPower(-gamepad1.right_stick_x / 2);
                } } else {
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
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


