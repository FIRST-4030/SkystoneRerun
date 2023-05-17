package org.firstinspires.ftc.teamcode.main.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.general.input.DSController;
import org.firstinspires.ftc.teamcode.util.general.misc.GeneralConstants;

/**
 * This class is primarily used to test if encoders on the motors work properly and report correctly to roadrunner systems
 * <br>To use it, run the OpMode and push the robot around and you will see what would roadrunner sees as encoder positions
 */
@Config
@TeleOp(group = GeneralConstants.TEST_OPMODE)
public class WheelMotorEncoderTest extends LinearOpMode {

    private SampleMecanumDrive drive;
    public DSController inputHandler;
    private static DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public static double drivePower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        inputHandler = new DSController(gamepad1);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LR");
        rightRear = hardwareMap.get(DcMotorEx.class, "RR");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        //enable BULK READ to mimic how roadrunner reads the values
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            multipleTelemetry.addData("Left Front Motor Encoder Position: ", leftFront.getCurrentPosition());
            multipleTelemetry.addData("Left Rear Motor Encoder Position: ", leftRear.getCurrentPosition());
            multipleTelemetry.addData("Right Front Motor Encoder Position: ", rightFront.getCurrentPosition());
            multipleTelemetry.addData("Right Rear Motor Encoder Position: ", rightRear.getCurrentPosition());
            multipleTelemetry.update();

            inputHandler.run();
            handleDriver();
        }
    }

    public void handleDriver(){
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -inputHandler.leftStickY.getValue(),
                -inputHandler.leftStickX.getValue()
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * drivePower,
                        input.getY() * drivePower,
                        -inputHandler.rightStickX.getValue()
                )
        );
    }
}
