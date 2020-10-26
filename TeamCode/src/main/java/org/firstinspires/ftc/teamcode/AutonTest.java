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


import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Autonomous(name = "AutonTest")
public class AutonTest extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    //private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ATOQy0T/////AAABmQ/SASRSuUzAugTNY1JUz6NSX/K0IyvPgOTMGlPcjAUyxoQlqULPX1jcW4C4fMzALWwznPmVdS4QFyFERfGevgAJPX1U8c6c1wfPTrhaqwhhoG0SBo/8b6iGaeweb65NN1Xu7PG+LHieN8rr339wsfsGROM9TW3oRp6uYdDFq30aM9tAIadifcbtQq9XUSdUyzF7Owgr8QjIbAw57OYIb6Bwl+7tenxUVSM+pYEkZyVdbCWAWEHAeu10tX7qGXmPEENNiKfnTT/TvGCsjTVK0Xa536mx70V74J/wBadTw32md6QMKjQmWWtCqyYnYZenYws42NGZzXsD/cLU+IbXOIJBaohDU8SaPr7mienrXGgc";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    //Function that takes power and distance and goes forward that distance
    public void driveForwardDistance(double power, int distance) {

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        long ticks1 = Math.round(distance/0.0229);
        int ticks2 = (int)ticks1;


        motorFrontRight.setTargetPosition(-ticks2);
        motorFrontLeft.setTargetPosition(-ticks2);
        motorBackRight.setTargetPosition(-ticks2);
        motorBackLeft.setTargetPosition(-ticks2);

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

    //fuction that turns
    public void turn(double power, double angle) {


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);


        motorFrontRight.setTargetPosition((int) ((((3600 / 360) * angle))));
        motorBackRight.setTargetPosition((int) ((((3600 / 360) * angle))));
        motorFrontLeft.setTargetPosition((int) ((-((3600 / 360) * angle))));
        motorBackLeft.setTargetPosition((int) ((-((3600 / 360) * angle))));

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


    public void strafe(double power, int distance) {

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        long ticks1 = Math.round(distance/0.0229);
        int ticks2 = (int)ticks1;

        motorFrontRight.setTargetPosition(-ticks2);
        motorFrontLeft.setTargetPosition(ticks2);
        motorBackRight.setTargetPosition(ticks2);
        motorBackLeft.setTargetPosition(-ticks2);

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

    //set starting position
    double currentX = 34;
    double currentY = 0;


    public void xfGoTo(double x, double y){
        if (x > currentX || x < currentX){
            long changeX1 = Math.round(x - currentX);
            int changeX2 = (int)changeX1;
            strafe(0.5, changeX2);
        }
        if (y > currentY || y < currentY){
            long changeY1 = Math.round(y - currentY);
            int changeY2 = (int)changeY1;
            driveForwardDistance(0.5, changeY2);
        }

        currentX = x;
        currentY = y;

    }

    public void yfGoTo(double x, double y){

        if (y > currentY || y < currentY){
            long changeY1 = Math.round(y - currentY);
            int changeY2 = (int)changeY1;
            driveForwardDistance(0.5, changeY2);
        }

        if (x > currentX || x < currentX){
            long changeX1 = Math.round(x - currentX);
            int changeX2 = (int)changeX1;
            strafe(0.5, changeX2);
        }

        currentX = x;
        currentY = y;

    }

    boolean isRunning = true;
    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();


            tfod.setZoom(1.5, 1.78);
        }

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        int rings = 0;


        waitForStart();
        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        */

        //detect using camera
        boolean ison = true;


            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    while (isRunning == true) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                if (recognition.getLabel().equals(null)) {
                                    rings = 0;
                                    isRunning = false;
                                } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                    rings = 1;
                                    isRunning = false;
                                } else if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                    rings = 4;
                                    isRunning = false;
                                }
                            }
                            telemetry.addData("rings = ", rings);
                            telemetry.update();
                        }
                    }
                }

                    if (tfod != null) {
                        tfod.shutdown();
                    }

                    xfGoTo(52, 50);
                    sleep(200);
                    xfGoTo(44.5, 50);
                    sleep(200);
                    xfGoTo(37, 50);
                    sleep(200);

                    stop();
                }
        }










    }

            private void initVuforia() {

                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                parameters.vuforiaLicenseKey = VUFORIA_KEY;
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

                //  Instantiate the Vuforia engine
                vuforia = ClassFactory.getInstance().createVuforia(parameters);

                // Loading trackables is not necessary for the TensorFlow Object Detection engine.
            }

            private void initTfod(){
                int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfodParameters.minResultConfidence = 0.8f;
                tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
                tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

            }











}
