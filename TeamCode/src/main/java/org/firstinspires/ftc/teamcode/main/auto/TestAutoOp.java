package org.firstinspires.ftc.teamcode.main.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.main.testing.HookHandler;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.MecanumEndpoint;
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

    public static Pose2dWrapper startPose = new Pose2dWrapper(-36, -64, 1.5707);
    public static MecanumEndpoint PLAT_POINT = new MecanumEndpoint( -60, -30, 1.5707, 30, 30);
    public static MecanumEndpoint PLAT_POINT_2 = new MecanumEndpoint( -32, -45, 3.14159);

    public static MecanumEndpoint MID_POINT = new MecanumEndpoint(0, -40, 3.14159);

    public static MecanumEndpoint BLOCK_POINT = new MecanumEndpoint( 11, -32, 3.534);
    public static MecanumEndpoint BLOCK_POINT_2 = new MecanumEndpoint(18, -26, 3.534);
    public static MecanumEndpoint BLOCK_POINT_3 = new MecanumEndpoint(27, -32, 3.534);
    public static MecanumEndpoint BLOCK_POINT_4 = new MecanumEndpoint(35, -32, 3.534);

    //public static double SPLINE_MAX_VEL = 30, SPLINE_MAX_ACCEL = 30;
    public Servo claw;
    public Servo HR;
    public Servo HL;
    public static HookHandler hookController;




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

        claw = hardwareMap.servo.get("claw");
        HR = hardwareMap.servo.get("HR");
        HL = hardwareMap.servo.get("HL");
        hookController = new HookHandler();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose.toPose2d());

        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(PLAT_POINT.pose.x, PLAT_POINT.pose.y), PLAT_POINT.pose.heading, PLAT_POINT.getMaxVel(), PLAT_POINT.getMaxAccel())
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(PLAT_POINT_2.getPose())
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(MID_POINT.getPose())
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(BLOCK_POINT.getPose())
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(MID_POINT.getPose())
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(PLAT_POINT_2.getPose())
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToLinearHeading(MID_POINT.getPose())
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToLinearHeading(BLOCK_POINT_2.getPose())
                .build();


        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1, traj2, traj4, traj5, traj6, traj7, traj8);

        waitForStart();

        drive.followTrajectory(traj1);
        handleHooks(true);
        sleep(250);
        drive.followTrajectory(traj2);
        handleHooks(false);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
    }
    public void handleHooks(boolean hookBool){
        //Hook Control
        double[] hookPosititon = hookController.updatePosition(hookBool);

        HL.setPosition(hookPosititon[0]);
        HR.setPosition(hookPosititon[1]);

        /*
        if (hookBool) HL.setPosition(0.75);
        if (!hookBool) HL.setPosition(0.2);

        HR.setPosition(1-0.2-HL.getPosition());

         */
    }
}
