package org.firstinspires.ftc.teamcode.main.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.general.misc.GeneralConstants;
import org.firstinspires.ftc.teamcode.util.general.subsystems.lift.LiftController;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.general.input.DSController;

@TeleOp(group = GeneralConstants.SAMPLE_OPMODE)
public class SimpleMecanumDrive extends OpMode {

    private SampleMecanumDrive drive;
    public DSController inputHandler;

    public Servo hookLeft;
    public Servo hookRight;
    public boolean isHookDown = false;
    public int incorrectDown = 10;
    public int incorrectUp = 120;

    public LiftController lift;
    public String motorName = "liftMotor";
    public State.Sequence liftMachine;
    private int liftPos = 0;




    @Override
    public void init() {
        inputHandler = new DSController(gamepad1);

        hookLeft = hardwareMap.servo.get("leftHook");
        hookRight = hardwareMap.servo.get("rightHook");

        lift = new LiftController(hardwareMap, motorName, false);
        liftMachine = new State.Sequence();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
    }

    @Override
    public void loop() {
        inputHandler.run(); //input
        handleDrive(); //drive
        handleLift(); //lift
        handleHooks(); //hooks
    }

    public void handleDrive(){
        //Field-centric drive in a normal OpMode
        //Src: https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpFieldCentric.java

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
                        input.getX(),
                        input.getY(),
                        -inputHandler.rightStickX.getValue()
                )
        );
    }

    public void handleLift(){
        //Lift Control
        if (gamepad1.dpad_up) liftPos += 1;
        if (gamepad1.dpad_down) liftPos -= 1;
        lift.setTargetPosition(liftPos);
        lift.update();
    }

    public void handleHooks(){
        //Hook Control
        if (inputHandler.buttonA.pressed) isHookDown = !isHookDown;

        if (isHookDown) hookLeft.setPosition(incorrectDown);
        if (!isHookDown) hookLeft.setPosition(incorrectUp);
        hookRight.setPosition(hookLeft.getPosition());
    }
}
