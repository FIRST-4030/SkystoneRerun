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
    public static double x1 = 30;
    public static double x2 = 80;
    public static double y1 = 30;
    public static double y2 = -10;

    public static double endX = 0;
    public static double endY = -15;
    public static double endHeading = 180;

    public static double heading1 = 45;
    public static double heading2 = 270;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x1, y1), Math.toRadians(heading1))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(x2, y2), Math.toRadians(heading2))
                .build();



        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj);
        sleep(waitTimeMs);
        drive.followTrajectory(traj2);

        sleep(waitTimeMs);

        if (reverseSpline) {
            drive.followTrajectory(
                    drive.trajectoryBuilder(traj2.end(), false)
                            .splineTo(new Vector2d(endX, endY), Math.toRadians(endHeading))
                            .build()
            );
        }
    }
}
