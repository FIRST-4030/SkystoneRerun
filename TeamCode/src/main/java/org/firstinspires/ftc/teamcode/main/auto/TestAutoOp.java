package org.firstinspires.ftc.teamcode.main.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    public static Pose2dWrapper startPose = new Pose2dWrapper(-62, -11.5, 0);
    public static SplineConstants PLAT_POINT = new SplineConstants( -24.5, -47.75, 0,0);
    public static SplineConstants PLAT_POINT_HALF = new SplineConstants( -60, -45, Math.toRadians(270), 0);


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
                .splineTo(new Vector2d(PLAT_POINT_HALF.x, PLAT_POINT_HALF.y), Math.toRadians(PLAT_POINT_HALF.heading))
                .splineTo(new Vector2d(PLAT_POINT.x, PLAT_POINT.y), Math.toRadians(PLAT_POINT.heading))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(PLAT_POINT_HALF.x, PLAT_POINT_HALF.y))
                .build();


        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1);

        DriveCmdMaker.init(drive);
        DriveCmdMaker.getInstance()
                .addDriveCmd(traj1)
                .addIntermediateState(new State.Wait(1000)) //pause one second
                .addDriveCmd(traj2);
        driveSequence = DriveCmdMaker.getInstance().build();

        waitForStart();
        driveSequence.init();
        while (!isStopRequested()){
            driveSequence.run();
        }
        driveSequence.end();
        //drive.followTrajectory(traj); //run trajectory ONCE
    }
}
