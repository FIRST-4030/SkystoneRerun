package org.firstinspires.ftc.teamcode.main.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.general.misc.GeneralConstants;
import org.firstinspires.ftc.teamcode.util.general.lift.LiftController;
import org.firstinspires.ftc.teamcode.util.general.lift.LiftCmd;
import org.firstinspires.ftc.teamcode.util.statemachine.State;

@TeleOp(group = GeneralConstants.SAMPLE_OPMODE)
public class SimpleMecanumDrive extends OpMode {

    private SampleMecanumDrive drive;
    public LiftController lift;
    public String motorName = "motor";
    public State.Sequence liftMachine;


    @Override
    public void init() {
        lift = new LiftController(hardwareMap, motorName, false);
        liftMachine = new State.Sequence();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
    }

    @Override
    public void loop() {
        //Field-centric drive in a normal OpMode
        //Src: https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpFieldCentric.java

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );

        //Lift Control
        lift.update();
        liftMachine.run();

    }

}
