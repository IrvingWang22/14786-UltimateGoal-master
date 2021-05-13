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
@Autonomous(name = "BlueAuton")
public class BlueAuton extends NextAuton {

    public static Positions pos = new Positions();

    private boolean autonRunning = true;

    Trajectories traj = new Trajectories(super.drive, super.mech, super.startPose, pos.BLUE_JUNCTION_X, pos.BLUE_JUNCTION_Y, pos.BLUE_WOBBLE_X_0, pos.BLUE_WOBBLE_Y_0, pos.BLUE_SHOOTING_X, pos.BLUE_SHOOTING_X, pos.BLUE_ENDING_X, pos.BLUE_ENDING_Y);

    public BlueAuton() {
        super(pos.BLUE_STARTING_X, pos.BLUE_STARTING_Y);
    }
    double wobbleX;
    double wobbleY;

    @Override
    public void runOpMode() {
        ObjectDetection od = new ObjectDetection(hardwareMap);
        super.runOpMode();

        if (opModeIsActive()) {
            while (opModeIsActive() && autonRunning) {
                if (od.tfod != null) {
                    List<Recognition> updatedRecognitions = od.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            od.data(i, recognition, telemetry);
                            i++;
                            if (recognition.getLabel().equals("Quad")) {
                                // QUAD RINGS
                                telemetry.addData("QUAD FOUND", recognition.getConfidence());
                                wobbleX = pos.RED_WOBBLE_X_4;
                                wobbleY = pos.RED_WOBBLE_Y_4;

                                //
                                // traj = new Trajectories(wobbleX,wobbleY);
                                break;
                            } else if (recognition.getLabel().equals("Single")) {
                                // SINGLE RING
                                telemetry.addData("SINGLE FOUND", recognition.getConfidence());
                                traj = new Trajectories(super.drive, super.mech, super.startPose, pos.BLUE_JUNCTION_X, pos.BLUE_JUNCTION_Y, pos.BLUE_WOBBLE_X_1, pos.BLUE_WOBBLE_Y_1, pos.BLUE_SHOOTING_X, pos.BLUE_SHOOTING_X, pos.BLUE_ENDING_X, pos.BLUE_ENDING_Y);
                                break;
                            } else {
                                // NO RINGS
                                telemetry.addLine("NONE FOUND");
                                traj = new Trajectories(super.drive, super.mech, super.startPose, pos.BLUE_JUNCTION_X, pos.BLUE_JUNCTION_Y, pos.BLUE_WOBBLE_X_0, pos.BLUE_WOBBLE_Y_0, pos.BLUE_SHOOTING_X, pos.BLUE_SHOOTING_X, pos.BLUE_ENDING_X, pos.BLUE_ENDING_Y);
                                break;
                            }

                        }
                        telemetry.update();

                        // RUN TRAJECTORIES
                        super.runTrajFLIP(traj);

                        autonRunning = false;
                    }
                }
            }
        }

        od.shutdown();
    }
}