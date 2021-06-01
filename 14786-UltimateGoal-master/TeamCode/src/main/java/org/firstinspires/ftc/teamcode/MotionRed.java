package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.nextcore.Mechanisms;

@Config
@Autonomous(name = "A - Motion Red")
public class MotionRed extends LinearOpMode {

    // CHANGE FOR WOBBLE
    public static int WOBBLE_OPTION = 0; // 0,1,4


    // FIELD SPECIFIC DATA
    public static double MAX_VALUE = 72;
    public double MIN_VALUE = -MAX_VALUE;

    public static double TILE = 24;
    public static double HALF_TILE = TILE / 2;
    public static double ONE_HALF_TILE = TILE + HALF_TILE;

    public static double ANGLE_SCALE = 16 / 15;

    public static double TAPE_WIDTH = 2;

    // launch line ~ 80 from audience perimeter wall, tape width is 2, want to be on the middle
    public static double LAUNCH_LINE_X = 80 - MAX_VALUE + (TAPE_WIDTH / 2);

    // ROBOT SPECIFIC DATA
    public static double ROBOT_LENGTH = 18; // LENGTH OF WHEELS SIDE
    public static double ROBOT_WIDTH = ROBOT_LENGTH; // LENGTH OF INTAKE SIDE

    // RED SPECIFIC DATA
    public static double RED_STARTING_X = -63;  // STARTING X FOR AUTON
    public static double RED_STARTING_Y = -25.5;  // STARTING Y FOR AUTON

    public static double RED_JUNCTION_X = -12;
    public static double RED_JUNCTION_Y = -12;

    public static double RED_JUNCTION_X2 = 36;
    public static double RED_JUNCTION_Y2 =  0;

    public static double RED_SHOOTING_X = -1;
    public static double RED_SHOOTING_Y = -39;

    public static double RED_ENDING_X = LAUNCH_LINE_X; // STARTING X FOR TELEOP + ENDING X FOR AUTON
    public static double RED_ENDING_Y = RED_SHOOTING_Y; // STARTING Y FOR TELEOP + ENDING Y FOR AUTON

    public static double RED_WOBBLE_X_0 = 0;
    public static double RED_WOBBLE_Y_0 = -48;

    public static double RED_WOBBLE_X_1 = 24; //36
    public static double RED_WOBBLE_Y_1 = -24; //-12

    public static double RED_WOBBLE_X_4 = 48; //60
    public static double RED_WOBBLE_Y_4 = -48; //-12

    public double RED_WOBBLE_X, RED_WOBBLE_Y;

    public static double RED_SECOND_WOBBLE_X = -TILE*2;
    public static double RED_SECOND_WOBBLE_Y = -ONE_HALF_TILE; //36

    // from left to right
    public static double RED_POWERSHOT_X = RED_SHOOTING_X;

    public static double RED_POWERSHOT_Y_1 = -31;
    public static double RED_POWERSHOT_Y_2 = -37;
    public static double RED_POWERSHOT_Y_3 = -42;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(hardwareMap);

        Pose2d startPose = new Pose2d(RED_STARTING_X, RED_STARTING_Y, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        if (WOBBLE_OPTION == 0) {
            RED_WOBBLE_X = RED_WOBBLE_X_0;
            RED_WOBBLE_Y = RED_WOBBLE_Y_0;
        } else if (WOBBLE_OPTION == 1) {
            RED_WOBBLE_X = RED_WOBBLE_X_1;
            RED_WOBBLE_Y = RED_WOBBLE_Y_1;
        } else {
            RED_WOBBLE_X = RED_WOBBLE_X_4;
            RED_WOBBLE_Y = RED_WOBBLE_Y_4;
        }

        Trajectory dropWobble = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(RED_JUNCTION_X, RED_JUNCTION_Y), Math.toRadians(0))
                .splineTo(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y),Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    mech.setShooter(Mechanisms.motorPower.HIGH);
                })
                .build();


        Trajectory shootRings1 = drive.trajectoryBuilder(dropWobble.end())
                //.splineToConstantHeading(new Vector2d(RED_JUNCTION_X2, RED_JUNCTION_Y2), Math.toRadians(0))
                .splineTo(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y), Math.toRadians(0))
                .build();


        Trajectory intakeRings = drive.trajectoryBuilder(shootRings1.end())
                .addDisplacementMarker(() -> {
                    mech.runIntake(Mechanisms.intakeState.IN);
                })
                .back(2 * TILE)
                .build();

        Trajectory shootRings2 = drive.trajectoryBuilder(intakeRings.end())
                .splineToConstantHeading(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y), Math.toRadians(0))
                .build();
                        /*
                        Trajectory getSecondWobble = drive.trajectoryBuilder(shootRings2.end())
                                .splineToConstantHeading(new Vector2d(RED_JUNCTION_X3, RED_JUNCTION_Y3), Math.toRadians(0))
                                .build();

                        Trajectory getSecondWobble2 = drive.trajectoryBuilder(getSecondWobble.end())
                                .strafeRight(20)
                                .build();

                         Trajectory dropSecondWobble = drive.trajectoryBuilder(getSecondWobble2.end())
                                .splineToConstantHeading(new Vector2d(RED_JUNCTION_X2, RED_JUNCTION_Y2), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y), Math.toRadians(0))
                                .build();
                        */
        Trajectory park = drive.trajectoryBuilder(shootRings2.end())
                .addDisplacementMarker(() -> {
                    mech.setShooter(Mechanisms.motorPower.OFF);
                })
                .splineToConstantHeading(new Vector2d(RED_ENDING_X, RED_ENDING_Y), Math.toRadians(0))
                .build();

        drive.followTrajectory(dropWobble);

        mech.wait(500);
        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
        mech.wait(500);
        mech.wobbleControl(Mechanisms.wobblePos.OPEN);
        mech.wait(500);
        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
        mech.wait(500);

        drive.followTrajectory(shootRings1);

        mech.wait(500);
        mech.pushRings();
        mech.wait(500);

        drive.followTrajectory(intakeRings);

        mech.wait(500);
        mech.runIntake(Mechanisms.intakeState.OFF);

        drive.followTrajectory(shootRings2);

        mech.wait(500);
        mech.pushRings();
        mech.wait(500);
    }
}