package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.nextcore.Mechanisms;
import org.firstinspires.ftc.teamcode.nextcore.NextAuton;
import org.firstinspires.ftc.teamcode.nextcore.ObjectDetection;
import org.firstinspires.ftc.teamcode.nextcore.Positions;
import org.firstinspires.ftc.teamcode.nextcore.Trajectories;

import java.util.List;

@Config
@Autonomous(name = "ParkBlue")
public class ParkBlue extends NextAuton {

    public static Positions pos = new Positions();

    private boolean autonRunning = true;

    Trajectories traj = new Trajectories(super.drive, super.mech, super.startPose, pos.BLUE_JUNCTION_X, pos.BLUE_JUNCTION_Y, pos.BLUE_WOBBLE_X_0, pos.BLUE_WOBBLE_Y_0, pos.BLUE_SHOOTING_X, pos.BLUE_SHOOTING_X, pos.BLUE_ENDING_X, pos.BLUE_ENDING_Y);

    public ParkBlue() {
        super(pos.BLUE_STARTING_X, pos.BLUE_STARTING_Y);
    }

    @Override
    public void runOpMode() {
        ObjectDetection od = new ObjectDetection(hardwareMap);
        super.runOpMode();

        if (opModeIsActive()) {
            /*
            mech.wait(15000);
            Trajectory parkBlue = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(pos.BLUE_ENDING_X, pos.BLUE_ENDING_Y)
                            .build()
            */
        }

        od.shutdown();
    }
}