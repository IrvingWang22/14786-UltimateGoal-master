package org.firstinspires.ftc.teamcode.nextcore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@Config
public class  Mechanisms {
    // Init Objects
    public ElapsedTime runtime = new ElapsedTime();

    // Init Objects: DcMotorEx
    public DcMotorEx shooterOne, shooterTwo;

    public DcMotorEx intake;

    // Init Objects: CRServo
    private CRServo bottomRoller;

    // Init Objects: Servo
    public Servo indexPush;

    private Servo wobbleGrab, wobbleArm, wobbleTurret;

    public Servo stickOne, stickTwo;

    // Init Lists

    // Other Variables
    public static int PUSH_RESTORE_TIME = 100;

    // Servo Positions
    public static double PUSH_MAX_VALUE = 0.29;
    public static double PUSH_MIN_VALUE = 0.45;

    public static double WOBBLE_MAX_VALUE = 1.0;
    public static double WOBBLE_INIT_VALUE = 0.2;
    public static double WOBBLE_MIN_VALUE = 0.58;

    public static double WOBBLE_CLAW_MIN_VALUE = 0.5;
    public static double WOBBLE_CLAW_MAX_VALUE = 0.1;

    public static double WOBBLE_TURRET_MAX_VALUE = 0.87;
    public static double WOBBLE_TURRET_MIN_VALUE = 0.1;

    public static double STICK_MAX_VALUE = 0.5;
    public static double STICK_MIN_VALUE = 1;

    public static double SHOOT_TPS = 1600; //HIGH speed
    public static double POWERSHOT_TPS = 1400; //MED speed
    public static double MIDDLEGOAL_TPS = 1300; //LOW speed


    // Power Enum
    public enum motorPower {
        HIGH, STALL, OFF, MED, LOW
    }

    public enum intakeState {
        IN, OUT, OFF
    }

    // Wobble Enum
    public enum wobblePos {
        OPEN, CLOSE
    }

    public enum wobbleArmPos {
        UP, DOWN, INIT
    }

    public enum wobbleTurretPos {
        IN, OUT
    }

    public enum stickOnePos {
        UP, DOWN
    }

    public enum stickTwoPos {
        UP, DOWN
    }

    // Power Values
    public static double BOTTOM_ROLLER_POWER = 0.9;
    public static double INTAKE_POWER = 1;

    private static double OFF_POWER = 0;

    // Constructor
    public Mechanisms(HardwareMap hardwareMap) {
        //  Hardware mapping: DcMotorEx
        shooterOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        shooterTwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        //  Hardware mapping: CRServo
        bottomRoller = hardwareMap.get(CRServo.class, "bottomRoller");

        //  Hardware mapping: Servo
        indexPush = hardwareMap.get(Servo.class, "indexPush");

        wobbleGrab = hardwareMap.get(Servo.class, "wobbleGrab");
        wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");
        wobbleTurret = hardwareMap.get(Servo.class, "wobbleTurret");

        stickOne = hardwareMap.get(Servo.class, "stickOne");
        stickTwo = hardwareMap.get(Servo.class, "stickTwo");

        // Set Directions

        intake.setDirection(DcMotor.Direction.REVERSE);
        bottomRoller.setDirection(DcMotor.Direction.REVERSE);


        // Set modes

        shooterOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Init initial Positions
        indexPush.setPosition(PUSH_MIN_VALUE);

        wobbleControl(wobblePos.CLOSE);
        wobbleArmControl(wobbleArmPos.INIT);
        wobbleTurretControl(wobbleTurretPos.IN);


        //Set initial Position of Bars
        stickOne.setPosition(0.8);

    }

    // Set Shooter List to HIGH, STALL, OR OFF
    public void setShooter(motorPower power) {
        switch (power) {
            case HIGH:
                shooterOne.setVelocity(SHOOT_TPS);
                shooterTwo.setVelocity(SHOOT_TPS);
                break;
            case STALL:
                shooterOne.setVelocity(SHOOT_TPS / 2);
                shooterTwo.setVelocity(SHOOT_TPS / 2);
                break;
            case MED:
                shooterOne.setVelocity(POWERSHOT_TPS);
                shooterTwo.setVelocity(POWERSHOT_TPS);
                break;
            case LOW:
                shooterOne.setVelocity(MIDDLEGOAL_TPS);
                shooterTwo.setVelocity(MIDDLEGOAL_TPS);
                break;

            default:
                shooterOne.setVelocity(OFF_POWER);
                shooterTwo.setVelocity(OFF_POWER);

        }
    }


    // Hits rings three times into shooter
    public void pushRings() {
        for (int i = 0; i < 5; i++) {
            indexPush.setPosition(PUSH_MAX_VALUE);
            wait(PUSH_RESTORE_TIME);
            indexPush.setPosition(PUSH_MIN_VALUE);
            wait(PUSH_RESTORE_TIME);
        }
    }

    // Hits rings three times into shooter
    public void pushRing() {
        for (int i = 0; i < 1; i++) {
            indexPush.setPosition(PUSH_MAX_VALUE);
            wait(PUSH_RESTORE_TIME);
            indexPush.setPosition(PUSH_MIN_VALUE);
            wait(PUSH_RESTORE_TIME);
        }
    }


    public void wobbleControl(wobblePos pos) {
        switch (pos) {
            case OPEN:
                wobbleGrab.setPosition(WOBBLE_CLAW_MIN_VALUE);
                break;
            case CLOSE:
                wobbleGrab.setPosition(WOBBLE_CLAW_MAX_VALUE);
                break;
        }
    }

    public void wobbleArmControl(wobbleArmPos pos) {
        switch (pos) {
            case UP:
                wobbleArm.setPosition(WOBBLE_MIN_VALUE);
                break;
            case INIT:
                wobbleArm.setPosition(WOBBLE_INIT_VALUE);
                break;
            case DOWN:
                wobbleArm.setPosition(WOBBLE_MAX_VALUE);
                break;
        }
    }

    public void wobbleTurretControl(wobbleTurretPos pos) {
        switch (pos) {
            case IN:
                wobbleTurret.setPosition(WOBBLE_TURRET_MAX_VALUE);
                break;
            case OUT:
                wobbleTurret.setPosition((WOBBLE_TURRET_MIN_VALUE));
                break;
        }
    }

    public void stickOneControl(stickOnePos pos) {
        switch (pos) {
            case UP:
                stickOne.setPosition(STICK_MIN_VALUE);
                break;
            case DOWN:
                stickOne.setPosition(STICK_MAX_VALUE);
                break;
        }
    }

    public void stickTwoControl(stickTwoPos pos) {
        switch (pos) {
            case UP:
                stickTwo.setPosition(STICK_MIN_VALUE);
                break;
            case DOWN:
                stickTwo.setPosition(STICK_MAX_VALUE);
                break;
        }
    }


    public void runIntake(intakeState state) {
        switch (state) {
            case IN:
                intake.setPower(INTAKE_POWER);
                bottomRoller.setPower(BOTTOM_ROLLER_POWER);
                break;
            case OUT:
                intake.setPower(-INTAKE_POWER);
                bottomRoller.setPower(-BOTTOM_ROLLER_POWER);
                break;
            default:
                intake.setPower(OFF_POWER);
                bottomRoller.setPower(OFF_POWER);
        }
    }


    // Wait function that doesn't interrupt program runtime, uses elapsed time
    public void wait(int milliseconds) {
        double currTime = runtime.milliseconds();
        double waitUntil = currTime + milliseconds;
        while (runtime.milliseconds() < waitUntil) {
            // remain empty
        }
    }
}