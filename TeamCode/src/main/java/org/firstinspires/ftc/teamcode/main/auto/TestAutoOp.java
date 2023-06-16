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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.main.testing.HookHandler;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.roadrunner.util.MecanumEndpoint;
import org.firstinspires.ftc.teamcode.roadrunner.util.SplineConstants;
import org.firstinspires.ftc.teamcode.util.general.input.DSController;
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
    //Rotation is CCW and 0 is on the x axis

    public static Pose2dWrapper startPose = new Pose2dWrapper(-36, -62, 1.5707);
    public static MecanumEndpoint PLAT_POINT = new MecanumEndpoint( -56, -28, 1.5707, 30, 30);
    public static MecanumEndpoint PLAT_POINT_2 = new MecanumEndpoint( -40, -55, 3.14159);

    //public static MecanumEndpoint MID_POINT = new MecanumEndpoint(2, -38, 3.14159);

    public static MecanumEndpoint BLOCK_POINT = new MecanumEndpoint( 23, -24, 3.92);
    public static MecanumEndpoint BLOCK_POINT_2 = new MecanumEndpoint(20, -25, 3.534);
    //public static MecanumEndpoint BLOCK_POINT_3 = MID_POINT;
    public static MecanumEndpoint BLOCK_POINT_4 = new MecanumEndpoint(35, -26, 3.534);

    //public static double SPLINE_MAX_VEL = 30, SPLINE_MAX_ACCEL = 30;
    public Servo claw;
    public Servo swing;
    public Servo HR;
    public Servo HL;
    public DcMotor IntakeLeft;
    public DcMotor IntakeRight;
    public int sign;

    public static HookHandler hookController;

    public static double upperLimit = 0.6;
    public static double lowerLimit = 0.338;

    public static double delay1 = 100, delay2 = 1500, delay3 = 1250;

    public DSController inputHandler;

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
        inputHandler = new DSController(gamepad1);
        telemetry.addData("Right on DPAD for RED, Left on DPAD for BLUE", "");
        while (!isStarted()){
           inputHandler.run();
           telemetry.addData("Multiplier", sign);
           //if the side is blue
           if(inputHandler.dPadLeft.released) {
               sign = 1;
           } //if the side is red
             else if(inputHandler.dPadRight.released){
               sign = -1;
           }
        }

        Pose2dWrapper startPose = new Pose2dWrapper(-36, -62, 1.5707);
        MecanumEndpoint PLAT_POINT = new MecanumEndpoint( -56, -28, 1.5707, 30, 30);
        MecanumEndpoint PLAT_POINT_2 = new MecanumEndpoint( -40, -55, 3.14159);

        MecanumEndpoint MID_POINT = new MecanumEndpoint(2, -38, 3.14159);

        MecanumEndpoint BLOCK_POINT = new MecanumEndpoint( 23, -24, 3.92);
        MecanumEndpoint BLOCK_POINT_2 = new MecanumEndpoint(20, -25, 3.534);
        MecanumEndpoint BLOCK_POINT_3 = MID_POINT;
        MecanumEndpoint BLOCK_POINT_4 = new MecanumEndpoint(35, -26, 3.534);

        if (sign < 0){
            startPose.y *= -1;
            startPose.heading *= -1;

            PLAT_POINT.invertSides();
            PLAT_POINT_2.invertSides();
            MID_POINT.invertSides();
            BLOCK_POINT.invertSides();
            BLOCK_POINT_2.invertSides();
            BLOCK_POINT_3.invertSides();
            BLOCK_POINT_4.invertSides();
        }

        claw = hardwareMap.servo.get("claw");
        HR = hardwareMap.servo.get("HR");
        HL = hardwareMap.servo.get("HL");
        hookController = new HookHandler();
        IntakeLeft = hardwareMap.dcMotor.get("CL");
        IntakeRight = hardwareMap.dcMotor.get("CR");
        swing = hardwareMap.servo.get("swing");

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

        /*Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToLinearHeading(BLOCK_POINT_2.getPose())
                .build();
        */

        DashboardUtil.previewTrajectories(FtcDashboard.getInstance(), traj1, traj2, traj4, traj5, traj6, traj7);

        handleClaw(true);



        drive.followTrajectory(traj1);
        handleHooks(true);
        sleep(250);
        drive.followTrajectory(traj2);
        handleHooks(false);

        //Start collecting blocks
        //goto block
        handleIntake(true);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

        handleSwing(0.345);
        sleep(100);
        //handleIntake(true); //go to first block and turn on intake
        handleSwing(lowerLimit);
        sleep(100);
        handleClaw(false);

        //return to platform
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);

        sleep((long) delay1);

        handleSwing(upperLimit);

        sleep((long) delay2);

        handleClaw(true);
        sleep(200);
        handleSwing(0.345);
        handleIntake(false);
        sleep((long) delay3);
        drive.followTrajectory(traj7);


        //end of collecting first block

        //start of collecting second block
    }
    public void handleHooks(boolean hookBool){
        //Hook Control
        double[] hookPosition = hookController.updatePosition(hookBool);
        HL.setPosition(hookPosition[0]);
        HR.setPosition(hookPosition[1]);
    }

    public void handleIntake(boolean isOn){
        IntakeLeft.setPower(isOn ? 1 : 0);
        IntakeRight.setPower(-IntakeLeft.getPower());
    }

    public void handleClaw(boolean clawOpen){
        claw.setPosition(!clawOpen ? 1 : 0.5);
    }

    public void handleSwing(double position){
        double output = Math.max(Math.min(position, upperLimit), lowerLimit);

        swing.setPosition(output);
    }

}
