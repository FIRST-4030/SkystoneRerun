package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.general.misc.Pose2dWrapper;

@Config
@Autonomous(group = "drive")
public class PrecisionTest extends LinearOpMode {

    private SampleMecanumDrive drive;

    public static Pose2dWrapper startPose = new Pose2dWrapper(0, 0, 0);

    public static boolean halt = false;

    public static int TEST_PRECISION_DRIVE = 0;
    public static double
            BASIC_FORWARD1 = 10,
            BASIC_TURN1 = 0.785398163397,
            BASIC_BACKWARD1 = 5,

            BASIC_FORWARD2 = 15,
            BASIC_TURN2 = 0.523598775598;

    public static Pose2dWrapper
            endPose1 = new Pose2dWrapper(0, 5, 1),
            endPose2 = new Pose2dWrapper(10, 5, 1);


    @Override
    public void runOpMode() throws InterruptedException {

        //init
        setupDrive();

        waitForStart();
        //start

        while (!isStopRequested()){
            if (!halt) {
                switch (TEST_PRECISION_DRIVE) {
                    case 0:
                        testBasicDriveCmd();
                        break;
                    case 1:
                        testSplineDriveCmd();
                    default:
                }
            }
        }
    }

    public void setupDrive(){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose.toPose2d());
    }

    public void testBasicDriveCmd(){
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(BASIC_FORWARD1)
                .back(BASIC_BACKWARD1)
                .build();

        drive.turn(BASIC_TURN1);

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().vec(), BASIC_TURN1))
                .forward(BASIC_FORWARD2)
                .build();

        drive.turn(BASIC_TURN2);

        followTrajectory(traj1);
        followTrajectory(traj2);
    }

    public void testSplineDriveCmd(){
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(endPose1.toPose2d().vec(), endPose1.heading)
                .splineTo(endPose2.toPose2d().vec(), endPose2.heading)
                .build();

        followTrajectory(traj1);
    }

    public void followTrajectory(Trajectory trajectory){
        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), trajectory);
        drive.followTrajectory(trajectory);
    }
}
