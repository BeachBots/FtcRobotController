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
public class PID {
    private DcMotor shoot1;
    private DcMotor shoot2;

    final double default_kp = .0052;
    final double default_kd = .4;

    public void init(HardwareMap hardwaremap) {
        shoot1 = hardwaremap.dcMotor.get("shoot1");
        shoot2 = hardwaremap.dcMotor.get("shoot2");

        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    private final int NUM_PID_ADJUSTMENTS = 50; // number of adjustments before we move to READY
    private final int MS_BTWN_VEL_READINGS = 12;
    private final int NUM_VELOCITY_READINGS = 20;

    private double targetVelocity = 0.0;
    public double currentVelocity = 0.0;
    private double currentPower = 0.0;

    private long lastLoopTime = 0;
    private double previousMotor = 0.0;

    private double velocity_sum;

    private Queue<Double> velocity_accumulator = new LinkedList<Double>();

    private double previousError = 0.0;
    private int pid_adjust_count = 0;
    private long lastAdjustTime = 0;

    private enum PID_STATE {
        RUNNING,
        READY,
    }

    private PID.PID_STATE state1;

    public void start(double inTargetVelocity) {
        state1 = PID.PID_STATE.RUNNING;

        targetVelocity = inTargetVelocity;
        currentPower = targetVelocity / 12;  // 3.36
        shoot1.setPower(currentPower);
        shoot2.setPower(currentPower);

        lastLoopTime = System.currentTimeMillis();
        previousMotor = shoot1.getCurrentPosition();

        velocity_accumulator.clear();
        velocity_sum = 0;
        previousError = 0;
        pid_adjust_count = 0;
        lastAdjustTime = System.currentTimeMillis();
    }

    public void loop() {
        loop(default_kp, default_kd);
    }

    public void loop(double kp, double kd) {
        // Read velocity and calculate error; set motors
        final long currentTime = System.currentTimeMillis();
        if (currentTime - lastLoopTime < MS_BTWN_VEL_READINGS) {
            return;
        }

        // calculate velocity
        final int currentMotor = shoot1.getCurrentPosition();
        final double changeMotor = currentMotor - previousMotor;
        previousMotor = currentMotor;
        final double new_velocity = changeMotor / (currentTime - lastLoopTime);
        lastLoopTime = currentTime;

        velocity_accumulator.add(new_velocity);
        velocity_sum += new_velocity;

        if (velocity_accumulator.size() <= NUM_VELOCITY_READINGS) {
            return;
        }

        velocity_sum -= velocity_accumulator.remove();
        currentVelocity = velocity_sum / velocity_accumulator.size();

        final double error = targetVelocity - currentVelocity;
        final double p = kp * error;
        final double d = kd * (error - previousError) / (currentTime - lastAdjustTime);
        previousError = error;
        lastAdjustTime = currentTime;

        currentPower = p + d + currentPower;
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
