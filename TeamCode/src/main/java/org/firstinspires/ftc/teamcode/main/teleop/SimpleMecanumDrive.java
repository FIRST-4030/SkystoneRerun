package org.firstinspires.ftc.teamcode.main.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.general.misc.GeneralConstants;
import org.firstinspires.ftc.teamcode.util.general.subsystems.lift.LiftController;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.general.input.DSController;

@TeleOp(name = "RerunDrive", group = GeneralConstants.SAMPLE_OPMODE)
public class SimpleMecanumDrive extends OpMode {

    private SampleMecanumDrive drive;
    public DSController inputHandler;

    public Servo hookLeft;
    public Servo hookRight;
    public Servo claw;
    public Servo swing;
    public DcMotor IntakeLeft;
    public DcMotor IntakeRight;
    public DcMotor liftMotor;
    public boolean isHookDown = false;
    public boolean isIntaking = false;
    public double offset = 0.2;
    public double incorrectDown = 0.75;
    public double incorrectUp = 0.201;
    public boolean clawBool = false;
    public double swingInc = 0.01;

    public LiftController lift;
    public String motorName = "lift";
    public State.Sequence liftMachine;
    private int liftPos = 0;
    public static double speed = 1000;


    @Override
    public void init() {
        inputHandler = new DSController(gamepad1);

        hookLeft = hardwareMap.servo.get("HL");
        hookRight = hardwareMap.servo.get("HR");

        IntakeLeft = hardwareMap.dcMotor.get("CL");
        IntakeRight = hardwareMap.dcMotor.get("CR");
        liftMotor = hardwareMap.dcMotor.get(motorName);

        claw = hardwareMap.servo.get("claw");
        swing = hardwareMap.servo.get("swing");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new LiftController(hardwareMap, motorName, true, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        handleIntake(); //intake
        handleClaw(); //claw
        handleSwing(); //swing

        telemetry.addData("Lift Encoder: ", liftMotor.getCurrentPosition());
        telemetry.addData("Speed: ", speed);
        telemetry.addData("Target: ", liftPos);
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
        if (inputHandler.dPadUp.held) liftPos = (int) speed;
        if (inputHandler.dPadDown.held) liftPos = (int) -speed;
        //lift.goToPos(liftPos);
        //lift.setPower(liftPos);
        liftMotor.setPower(liftPos);

        if(!inputHandler.dPadDown.getState() || !inputHandler.dPadUp.getState()) liftPos = 0;
        //lift.update();
    }

    public void handleHooks(){
        //Hook Control
        if (inputHandler.buttonA.pressed) isHookDown = !isHookDown;


        if (isHookDown) hookLeft.setPosition(incorrectDown);
        if (!isHookDown) hookLeft.setPosition(incorrectUp);

        hookRight.setPosition(1-offset-hookLeft.getPosition());
    }

    public void handleIntake() {
        if (inputHandler.buttonB.pressed) isIntaking = !isIntaking;
        if (isIntaking) IntakeLeft.setPower(1);
        if (!isIntaking) IntakeLeft.setPower(0);
        IntakeRight.setPower(IntakeLeft.getPower()*-1);
    }

    public void handleClaw() {
        if(inputHandler.dPadLeft.released) {
            clawBool = !clawBool;
        }
        if(clawBool) {claw.setPosition(1);}
        else {claw.setPosition(0.5);}

    }
    public void handleSwing() {
        swing.setPosition(swing.getPosition() + inputHandler.rightStickY.getValue() * swingInc);
    }
}
