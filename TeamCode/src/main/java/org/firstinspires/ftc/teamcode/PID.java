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


//@TeleOp(name = "PID")
public class PID {
    private DcMotor shoot1;
    private DcMotor shoot2;

    public void init(HardwareMap hardwaremap) {
        shoot1 = hardwaremap.dcMotor.get("shoot1");
        shoot2 = hardwaremap.dcMotor.get("shoot2");

        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private final int NUM_PID_ADJUSTMENTS = 10;
    private final int MS_BTWN_VEL_READINGS = 12;
    private final int NUM_VELOCITY_READINGS = 25;  // TEST THIS NUMBER -- keep halving it

    private double targetVelocity = 0.0;
    public double currentVelocity = 0.0;
    private double currentPower = 0.0;
    private long async_prevTime = 0;
    private double async_motorPrev = 0.0;
    private double async_prevError = 0.0;
    private double velocity_accumulator = 0.0;
    private int velocity_reading_count = 0;
    private int pid_adjust_count = 0;

    private enum PID_STATE {
        RUNNING,
        READY,
    }

    private PID.PID_STATE state1;

    public void start(double inTargetVelocity) {
        state1 = PID.PID_STATE.RUNNING;

        targetVelocity = inTargetVelocity;
        currentPower = targetVelocity / 3.36;
        shoot1.setPower(currentPower);
        shoot2.setPower(currentPower);

        async_prevTime = System.currentTimeMillis();
        async_motorPrev = shoot1.getCurrentPosition(); // changed from Math.abs(shoot1.getCurrentPosition()
        async_prevError = 0;
        velocity_accumulator = 0.0;
        velocity_reading_count = 0;

        pid_adjust_count = 0;
    }

    public void loop() {
        // Read velocity and calculate error; set motors
        final long currentTime = System.currentTimeMillis();
        if (currentTime - async_prevTime < MS_BTWN_VEL_READINGS) {
            return;
        }

        // calculate velocity
        final int currentMotor = shoot1.getCurrentPosition(); // changed from Math.abs(shoot1.getCurrentPosition())
        final double changeMotor = currentMotor - async_motorPrev;
        async_motorPrev = currentMotor;
        final long changeTime = currentTime - async_prevTime;
        async_prevTime = currentTime;
        final double new_velocity = changeMotor / changeTime;
        velocity_accumulator += new_velocity;
        velocity_reading_count++;

        if (velocity_reading_count < NUM_VELOCITY_READINGS) {
            return;
        }

        currentVelocity = velocity_accumulator / velocity_reading_count;
        velocity_reading_count = 0;
        velocity_accumulator = 0.0;

        final double kp = 0.10;  // .10
        final double kd = 0.30;  // .30

        final double error = targetVelocity - currentVelocity;
        final double p = kp * error;
        final double d = kd * ((error - async_prevError) / changeTime);
        async_prevError = error;

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
