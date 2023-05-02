package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {

    public static boolean reverseSpline = true;
    public static int waitTimeMs = 1000;
    public static double distance = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(distance, distance), 0)
                .splineTo(new Vector2d(0, distance * 2), 0)
                .build();

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj);

        sleep(waitTimeMs);

        if (reverseSpline) {
            drive.followTrajectory(
                    drive.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );
        }
    }
}
