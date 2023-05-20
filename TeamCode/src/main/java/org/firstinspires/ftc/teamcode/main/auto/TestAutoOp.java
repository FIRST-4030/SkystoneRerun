package org.firstinspires.ftc.teamcode.main.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.SplineConstants;
import org.firstinspires.ftc.teamcode.util.general.misc.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.util.general.rrutil.drivecmdmaker.DriveCmdMaker;
import org.firstinspires.ftc.teamcode.util.statemachine.State;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@Autonomous(group = "drive")
public class TestAutoOp extends LinearOpMode {

    //IMPORTANT: X increases upwards, Y increases to the left

    public static Pose2dWrapper startPose = new Pose2dWrapper(-12, -64, 1.5707);
    public static SplineConstants PLAT_POINT = new SplineConstants( -60, -30, 1.5707,0);
    public static SplineConstants PLAT_POINT_2 = new SplineConstants( -32, -36, 3.14159,0);

    public static double SPLINE_MAX_VEL = 60, SPLINE_MAX_ACCEL = 30;


    //Sequence testing
    public static State.Sequence driveSequence;

    /*
    //    public static double x1 = 30;
    public static double x2 = 80;
    //    public static double y1 = 30;
    public static double y2 = -10;

    public static double endX = 0;
    public static double endY = -15;
    public static double endHeading = 180;

    //    public static double heading1 = 45;
    public static double heading2 = 270;

     */

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(PLAT_POINT.x, PLAT_POINT.y), PLAT_POINT.heading, new MecanumVelocityConstraint(SPLINE_MAX_VEL, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(SPLINE_MAX_ACCEL))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2dWrapper(PLAT_POINT_2.x, PLAT_POINT_2.y, PLAT_POINT_2.heading).toPose2d())
                .build();


        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1, traj2);

        waitForStart();
        //while (!isStopRequested()){

        //}
        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2);

        //drive.followTrajectory(traj); //run trajectory ONCE
    }
}
