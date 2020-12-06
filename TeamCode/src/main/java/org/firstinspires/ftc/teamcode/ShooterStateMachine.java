package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.logging.SocketHandler;


@TeleOp(name = "ShooterStateMachine")
public class ShooterStateMachine extends OpMode {
    public enum ShooterState {
        SHOOTER_IDLE,
        SHOOTER_WAITING1,
        SHOOTER_INDEXING,
        SHOOTER_WAITING2,
        SHOOTER_RETRACTING,
        SHOOTER_WAITING3
    };

    // The shooterState variable is declared out here
    // so its value persists between loop() calls

    ShooterState shooterState = ShooterState.SHOOTER_IDLE;

    public DcMotor shoot1;   // should this be private?
    public DcMotor shoot2;
    public Servo flick;
    public Servo stopper;
    public DcMotor intake;
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;

    public double flickExtend = 0;
    public double flickRetract = .5;
    public double stopperClosed = 1;  // Temp number
    public double stopperOpen = 0;  // Temp number

    private int shotCounter;        //This is how we'll keep track of the 3 rings we are firing
    private long shooterStartTime;         //This will set the timer
    private long shooterDeltaTime = System.currentTimeMillis();

public void init() {

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
        shoot1.setDirection(DcMotor.Direction.REVERSE);  // BETTER TO DO THIS HERE THAN SETTING
        shoot2.setDirection(DcMotor.Direction.REVERSE);  // MOTOR POWER TO A NEGATIVE NUMBER LATER

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flick.setPosition(flickRetract);
        stopper.setPosition(stopperClosed);

    }

    public void loop() {


           // TURN SHOOTER ON
            if (gamepad1.y) {
                shoot1.setPower(.66);
                shoot2.setPower(.66);
            }

            // TURN SHOOTER OFF
            if (gamepad1.b) {
                shoot1.setPower(0);
                shoot2.setPower(0);
            }

            switch (shooterState) {
                case SHOOTER_IDLE:
                    // Waiting for some input
                    if (gamepad1.a) {
                        shotCounter = 1;
                        stopper.setPosition(stopperOpen); //This opens the stopper
                        shooterStartTime = System.currentTimeMillis(); //Records current time
                        shooterState = ShooterState.SHOOTER_WAITING1; //Moves to first timer
                    }
                    break;
                case SHOOTER_WAITING1: //This gives time for stopper to get out of the way
                    shooterDeltaTime = System.currentTimeMillis() - shooterStartTime; //How much time has elapsed
                    if (shooterDeltaTime > 100) { //This is the timer. When time elapses, we move on...
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
                    if (shooterDeltaTime > 100) {
                        shooterState = ShooterState.SHOOTER_RETRACTING; //Moves us to Retracting state
                    }
                    break;
                case SHOOTER_RETRACTING:
                    flick.setPosition(flickRetract); //Retracts the flicker
                    if (shotCounter == 3) { //This will return us to idle after the 3rd shot
                        stopper.setPosition(stopperClosed);  //Closes the stopper. Might be too fast-- test this.
                        shooterState = ShooterState.SHOOTER_IDLE;
                        break;  //I'm not sure why this is needed here, but it didn't work without it
                    }
                    shooterStartTime = System.currentTimeMillis(); //Resets timer
                    shotCounter++; //Increases the shot counter
                    shooterState = ShooterState.SHOOTER_WAITING3; //Moves us to Waiting2 state
                    break;
                case SHOOTER_WAITING3:
                    shooterDeltaTime = System.currentTimeMillis() - shooterStartTime; //How much time has elapsed
                    if (shooterDeltaTime > 200) { //This is the timer between shots
                        shooterState = ShooterState.SHOOTER_INDEXING; //Returns our state to Indexing
                    }
                    break;
                default:
                    shooterState = ShooterState.SHOOTER_IDLE;
                    break; //may not be necessary
            }

    }
}