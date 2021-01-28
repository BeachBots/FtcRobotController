package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.logging.SocketHandler;

//@TeleOp(name = "ShooterStateMachine")
public class ShooterStateMachine {
    public enum ShooterState {
        SHOOTER_IDLE,
        SHOOTER_WAITING1,
        SHOOTER_INDEXING,
        SHOOTER_WAITING2,
        SHOOTER_RETRACTING,
        SHOOTER_WAITING3,
        SHOOTER_WAITING4,
    }

    ;

    // The shooterState variable is declared out here
    // so its value persists between loop() calls

    public ShooterState shooterState = ShooterState.SHOOTER_IDLE;

    private DcMotor shoot1;
    private DcMotor shoot2;
    public Servo flick;
    public Servo stopper;
    public DcMotor intake;

    public double flickExtend = 0.80;
    public double flickRetract = 0.48;
    public double stopperClosed = 0.20;
    public double stopperOpen = 0.40;

    private int shotCounter;        //This is how we'll keep track of the 3 rings we are firing
    private long shooterStartTime;         //This will set the timer
    private long shooterDeltaTime = System.currentTimeMillis();
    private int num_shots = 0;


    public void init(HardwareMap hardwaremap) {

        shoot1 = hardwaremap.dcMotor.get("shoot1");
        shoot2 = hardwaremap.dcMotor.get("shoot2");
        intake = hardwaremap.dcMotor.get("intake");
        flick = hardwaremap.servo.get("flick");
        stopper = hardwaremap.servo.get("stopper");

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flick.setPosition(flickRetract);
        stopper.setPosition(stopperClosed);

    }

    /*
        pinchy/grabber thing ~= flick
        arm that goes up and down ~= shooter

        grabber
            tell it to open/close just like flick goes from middle to side
            bool open or closed right now
        arm
            get position out of it - know if it's at the maximum height, or at the ground

        functions
            setTargetHeight(float height);
               set your arm's desired angle, and say run to position

            open/close grabber
                set grabber state directly cause it's instant

            isDone()
                return !arm.isBusy()

            while (!arm.isDone()){} <- waits until we move into position;
     */

    public void shoot(int num) {
        shooterState = ShooterState.SHOOTER_WAITING1;
        num_shots = num;
        shotCounter = 0;
    }

    public void loop() {

        switch (shooterState) {
            case SHOOTER_IDLE:
                shooterStartTime = System.currentTimeMillis();  // Added on 1/23
                break;
            case SHOOTER_WAITING1: //This gives time for stopper to get out of the way
                stopper.setPosition(stopperOpen);
                shooterDeltaTime = System.currentTimeMillis() - shooterStartTime; //How much time has elapsed
                if (shooterDeltaTime > 250) { //This is the timer. When time elapses, we move on...
                    shooterState = ShooterState.SHOOTER_INDEXING; //Moves us to Indexing state
                }
                break;
            case SHOOTER_INDEXING: //This fires the shot
                flick.setPosition(flickExtend); //INDEXING extends the flicker
                shooterStartTime = System.currentTimeMillis(); //Records current time
                shooterState = ShooterState.SHOOTER_WAITING2; //Moves to second timer
                break;
            case SHOOTER_WAITING2:
                shooterDeltaTime = System.currentTimeMillis() - shooterStartTime;
                if (shooterDeltaTime > 150) { //Pause before retracting
                    shooterState = ShooterState.SHOOTER_RETRACTING; //Moves us to Retracting state
                }
                break;
            case SHOOTER_RETRACTING:
                flick.setPosition(flickRetract); //Retracts the flicker
                shooterStartTime = System.currentTimeMillis(); //Resets timer
                shotCounter++; //Increases the shot counter
                if (shotCounter == num_shots) { //This will send us to timer before raising stopper
                    shooterState = ShooterState.SHOOTER_WAITING4;
                    break;
                }
                shooterState = ShooterState.SHOOTER_WAITING3; //Moves us to Waiting3 state
                break;
            case SHOOTER_WAITING3:
                shooterDeltaTime = System.currentTimeMillis() - shooterStartTime;
                if (shooterDeltaTime > 400) { //This is the timer between shots
                    shooterState = ShooterState.SHOOTER_INDEXING; //Returns our state to Indexing
                }
                break;
            case SHOOTER_WAITING4:
                shooterDeltaTime = System.currentTimeMillis() - shooterStartTime;
                if (shooterDeltaTime > 350) { //This is the timer before raising stopper
                    stopper.setPosition(stopperClosed);
                    shooterState = ShooterState.SHOOTER_IDLE;
                }
                break;
            default:
                shooterState = ShooterState.SHOOTER_IDLE;
                break;
        }

    }
}
