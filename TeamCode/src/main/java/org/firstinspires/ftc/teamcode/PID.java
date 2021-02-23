package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedList;
import java.util.Queue;
import java.util.logging.SocketHandler;


//@TeleOp(name = "PID")
@Config
public class PID {
    public DcMotor shoot1;   // these used to be private. now can be seen in FinalTeleOp
    public DcMotor shoot2;

    public static double kp = .0052; //.007; // .0052;
    public static double kd = 1.0; //.6; // .4;

    public void init(HardwareMap hardwaremap) {
        shoot1 = hardwaremap.dcMotor.get("shoot1");
        shoot2 = hardwaremap.dcMotor.get("shoot2");

        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public static int NUM_PID_ADJUSTMENTS = 175; // number of adjustments before we move to READY
    public static int MS_BTWN_VEL_READINGS = 12;
    public static int NUM_VELOCITY_READINGS = 20;
    public static double FEED_FORWARD = 0.5;
    // public static int PID_DELAY = 1000;

    private double targetVelocity = 0.0;
    public double currentVelocity = 0.0;
    public double currentPower = 0.0; // 3.374?

    private long lastLoopTime = 0;
    private double previousMotor = 0.0;

    private double velocity_sum;

    private Queue<Double> velocity_accumulator = new LinkedList<Double>();

    private double previousError = 0.0;
    private int pid_adjust_count = 0;
    private long lastAdjustTime = 0;
    // private long pidStartDelayTime = 0;  // THIS IS IF WE USE THE TIMER

    private enum PID_STATE {
        RUNNING,
        READY,
    }

    private PID.PID_STATE state1;

    public void start(double inTargetVelocity) {
        state1 = PID.PID_STATE.RUNNING;

        targetVelocity = inTargetVelocity;
        //currentPower = targetVelocity / FEED_FORWARD; //.563
        shoot1.setPower(currentPower);
        shoot2.setPower(currentPower);

    //  pidStartDelayTime = System.currentTimeMillis();
        lastLoopTime = System.currentTimeMillis();
        previousMotor = shoot1.getCurrentPosition();

        velocity_accumulator.clear();
        velocity_sum = 0;
        previousError = 0;
        pid_adjust_count = 0;
        lastAdjustTime = System.currentTimeMillis();

    }

    public void loop() {

//        // OPTIONAL TIMER -- wait predetermined time for motor to reach speed before starting PID
//        long currentTime = System.currentTimeMillis();
//        if (currentTime - pidStartDelayTime < PID_DELAY) {
//            return;
//        }

        // read and calculate velocity
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastLoopTime < MS_BTWN_VEL_READINGS) {
            return;
        }

        final int currentMotor = shoot1.getCurrentPosition();
        final double changeMotor = currentMotor - previousMotor;
        previousMotor = currentMotor;
        final double new_velocity = changeMotor / (currentTime - lastLoopTime);
        lastLoopTime = currentTime;

        velocity_accumulator.add(new_velocity);
        velocity_sum += new_velocity;

        if (velocity_accumulator.size() < NUM_VELOCITY_READINGS) {
            return;
        }

        currentVelocity = velocity_sum / velocity_accumulator.size();
        velocity_sum -= velocity_accumulator.remove();

        // calculate error
        final double error = targetVelocity - currentVelocity;
        final double p = kp * error;
        final double d = kd * (error - previousError) / (currentTime - lastAdjustTime);
        previousError = error;
        lastAdjustTime = currentTime;

        // set motor power
        currentPower = p + d + 0.5;
        shoot1.setPower(currentPower);
        shoot2.setPower(currentPower);

        // Update Telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("power", currentPower);
        dashboardTelemetry.addData("kp", kp);
        dashboardTelemetry.addData("kd", kd);
        dashboardTelemetry.addData("velocity", currentVelocity);
        dashboardTelemetry.addData("target velocity", targetVelocity);
        dashboardTelemetry.update();

        // Update state
        if (++pid_adjust_count >= NUM_PID_ADJUSTMENTS) {
            state1 = PID.PID_STATE.READY;
        }
    }

    public boolean ready() {
        return state1 == PID.PID_STATE.READY;
    }

}
