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

import java.util.logging.SocketHandler;


@TeleOp(name = "PID")
public class PID {

    private DcMotor shoot1;
    private DcMotor shoot2;

    public void init(HardwareMap hardwaremap) {
        shoot1 = hardwaremap.dcMotor.get("shoot1");
        shoot2 = hardwaremap.dcMotor.get("shoot2");

    }

    private final int NUM_PID_ADJUSTMENTS = 10;
    private final int MS_BTWN_VEL_READINGS = 12;
    private final int NUM_VELOCITY_READINGS = 25;

    private double targetVelocity = 0.0;
    private double targetPower = 0.0;
    private long async_prevTime = 0;
    private double async_motorPrev = 0.0;
    private double async_prevError;
    private double velocity_accumulator = 0.0;
    private int velocity_reading_count = 0;
    private int pid_adjust_count = 0;


    private enum PID_STATE {
        RUNNING,
        DONE,
    }

    ;
    private PID.PID_STATE state1;


    public void start(double inTargetVelocity, double inPower) {
        state1 = PID.PID_STATE.RUNNING;


        targetVelocity = inTargetVelocity;
        targetPower = inPower;
        async_prevTime = System.currentTimeMillis();
        async_motorPrev = -shoot1.getCurrentPosition();
        async_prevError = 0;
        velocity_accumulator = 0.0;
        velocity_reading_count = 0;
        pid_adjust_count = 0;
    }

    public void loop() {
        // Read velocity and calculate error; set motors
        long currentTime = System.currentTimeMillis();
        if (currentTime - async_prevTime < MS_BTWN_VEL_READINGS) {
            return;
        }

        // calculate velocity
        final int currentMotor = shoot1.getCurrentPosition();
        final double changeMotor = currentMotor - async_motorPrev;
        final long changeTime = currentTime - async_prevTime;
        final double new_velocity = changeMotor / changeTime;
        velocity_reading_count++;
        velocity_accumulator += new_velocity;

        if (velocity_reading_count < NUM_VELOCITY_READINGS) {
            return;
        }

        final double currentVelocity = velocity_accumulator / velocity_reading_count;
        velocity_reading_count = 0;
        velocity_accumulator = 0.0;

        final double kp = 0.15;
        final double kd = 0.00;
        double error = targetVelocity - currentVelocity;
        final double p = kp * error;
        final double d = kd * ((error - async_prevError) / changeTime);
        shoot1.setPower(p + d + targetPower);
        shoot2.setPower(p + d + targetPower);

        // Update Telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("power", p + d + targetPower);
        dashboardTelemetry.addData("p", p);
        dashboardTelemetry.addData("d", d);
        dashboardTelemetry.addData("velocity", currentVelocity);
        dashboardTelemetry.update();

        // Update state
        targetPower = p + d + targetPower;
        async_prevError = error;
        async_prevTime = currentTime;
        async_motorPrev = currentMotor;

        if (pid_adjust_count++ >= NUM_PID_ADJUSTMENTS) {
            state1 = PID.PID_STATE.DONE;
        }
    }

    public boolean done() {
        return state1 == PID.PID_STATE.DONE;
    }

}
