package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import  com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotor shoot1;
    private DcMotor shoot2;
    private Servo flick;
    private Servo stopper;
    private DcMotor intake;
    private BNO055IMU imu;
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

    long prevTime = 0;
    long currentTime = 0;
    double motorCurrent = 0;
    double motorPrev = 0;

    public double getVelocity(){
        long changeTime = currentTime - prevTime;
        double changeMotor = motorCurrent - motorPrev;
        double velocity = changeMotor/changeTime;
        return velocity;
    }

    public void turn(double power, double angle){


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);


        motorFrontRight.setTargetPosition((int)((((3596/360) * angle))));
        motorBackRight.setTargetPosition((int)((((3596/360) * angle))));
        motorFrontLeft.setTargetPosition((int)((-((3596/360) * angle))));
        motorBackLeft.setTargetPosition((int)((-((3596/360) * angle))));

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

    //fuction that turns
    public void turnright(double power, double angle) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Gyro = angles.firstAngle;

        telemetry.addData("gyro", Gyro);
        telemetry.update();
        sleep(400);
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        while (Gyro > angle - 3 && Gyro < angle + 3){
            Orientation angles1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Gyro = angles1.firstAngle;



            motorFrontRight.setPower(-power);
            motorFrontLeft.setPower(power);
            motorBackRight.setPower(-power);
            motorBackLeft.setPower(power);
        }



    }

    public void turnleft(double power, double angle) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Gyro = angles.firstAngle;


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        while (Gyro > angle - 3 && Gyro < angle + 3){
            Orientation angles1 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Gyro = angles1.firstAngle;

            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(-power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(-power);
        }



    }


    public void strafe(double power, int distance) {

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);


        motorFrontRight.setTargetPosition(-distance);
        motorFrontLeft.setTargetPosition(distance);
        motorBackRight.setTargetPosition(distance);
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

        int rings = 0;




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        //detect using camera
        boolean ison = true;



                    while (isStarted() == false){
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            if (updatedRecognitions.size() == 0){
                                rings = 0;
                            }
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
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
                    }}



        waitForStart();




                    flick.setPosition(0.5);
                while (opModeIsActive()) {





                    //shoot1.setPower(-0.68);
                    //shoot2.setPower(-0.68);
                    sleep(100);
                    driveForwardDistance(0.5, 100);
                    sleep(150);
                    strafe(0.5, 180);
                    sleep(100);
                    flick .setPosition(0);
                    sleep(100);
                    flick.setPosition(0.5);
                    sleep(150);
                    strafe(0.5, 225);
                    sleep(200);
                    flick .setPosition(0);
                    sleep(100);
                    flick.setPosition(0.5);
                    sleep(150);
                    strafe(0.5, 225);
                    //shoot1.setPower(-0.7);
                    //shoot2.setPower(-0.7);
                    sleep(200);
                    flick .setPosition(0);
                    sleep(100);
                    flick.setPosition(0.5);
                    sleep(150);

                    shoot1.setPower(0);
                    shoot2.setPower(0);

                    if (rings == 0) {
                        driveForwardDistance(0.5, 300);
                        sleep(100);
                        turn(0.5, -28);
                        sleep(100);
                        driveForwardDistance(0.5, 2300);
                        sleep(100);
                        turn(0.5, 28);
                        sleep(100);
                        strafe(0.5, 250);
                        driveForwardDistance(0.5, -2400);
                        sleep(100);
                        driveForwardDistance(0.5, 2400);
                        sleep(100);
                        strafe(0.5, -450);
                    }

                    if (rings == 1){
                        strafe(0.5, -850);
                        sleep(100);
                        driveForwardDistance(0.5, 750);
                        sleep(100);
                        strafe(0.5, 500);
                        turn(0.5, -6);
                        sleep(100);
                        driveForwardDistance(0.5, 2600);
                        sleep(100);
                        driveForwardDistance(0.5, -3000);
                        sleep(250);
                        strafe(0.5, -700);
                        sleep(250);
                        strafe(0.5, 700);
                        sleep(100);
                        driveForwardDistance(0.5, 3000);
                        sleep(100);
                        driveForwardDistance(0.5, -500);
                    }

                    if (rings == 4){
                        strafe(0.5, -850);
                        sleep(100);
                        driveForwardDistance(0.5, 750);
                        sleep(100);
                        strafe(0.5, 500);
                        turn(0.5, -6);
                        sleep(100);
                        driveForwardDistance(0.5, 3400);
                        sleep(100);
                        strafe(0.5, -900);
                        sleep(100);
                        strafe(0.5, 900);
                        sleep(100);
                        turn(0.5, -6);
                        sleep(100);
                        driveForwardDistance(0.5, -4000);
                        sleep(200);
                        strafe(0.5,-900);
                        sleep(100);
                        strafe(0.5, 900);
                        sleep(100);
                        driveForwardDistance(0.5, 3400);
                        sleep(100);
                        strafe(0.5, -1200);
                        sleep(100);
                        driveForwardDistance(0.5, -800);

                    }







                    stop();
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
