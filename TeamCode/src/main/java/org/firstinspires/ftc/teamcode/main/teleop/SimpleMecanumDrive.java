package org.firstinspires.ftc.teamcode.main.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.main.testing.HookHandler;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.general.misc.GeneralConstants;
import org.firstinspires.ftc.teamcode.util.general.input.DSController;

@Config
@TeleOp(name = "RerunDrive", group = GeneralConstants.SAMPLE_OPMODE)
public class SimpleMecanumDrive extends OpMode {

    private SampleMecanumDrive drive;

    public DSController inputHandler;
    public DSController inputHandler2;

    public Servo hookLeft;
    public Servo hookRight;
    public Servo claw;
    public Servo swing;
    public DcMotor IntakeLeft;
    public DcMotor IntakeRight;
    public DcMotor liftMotor;

    public boolean isHookDown = false;
    public boolean isIntaking = false;

    public static double hookOffset = 0.2;
    public static double hookDown = 0.75;
    public static double hookUp = 0.23;
    public boolean clawBool = false;

    public static double clawOpen = 1;
    public static double clawClose = 0.5;
    public static double swingIncrement = 0.0017;

    public String motorName = "lift";
    private int liftPos = 0;
    public static double liftSpeed = 1;
    public static double upperLimit = 0.6;
    public static double lowerLimit = 0.338;

    public MultipleTelemetry multiTelemetry;

    public static HookHandler hookController;

    public static boolean swingIgnoreLimits = false;

    @Override
    public void init() {
        inputHandler = new DSController(gamepad1);
        inputHandler2 = new DSController(gamepad2);

        hookLeft = hardwareMap.servo.get("HL");
        hookRight = hardwareMap.servo.get("HR");
        hookController = new HookHandler();

        IntakeLeft = hardwareMap.dcMotor.get("CL");
        IntakeRight = hardwareMap.dcMotor.get("CR");
        liftMotor = hardwareMap.dcMotor.get(motorName);

        claw = hardwareMap.servo.get("claw");
        swing = hardwareMap.servo.get("swing");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        inputHandler.run(); //controller 1
        inputHandler2.run(); //controller 2

        handleDrive(); //drive
        handleLift(); //lift
        handleHooks(); //hooks
        handleIntake(); //intake
        handleClaw(); //claw
        handleSwing(); //swing

        multiTelemetry.addData("Lift Encoder: ", liftMotor.getCurrentPosition());
        multiTelemetry.addData("Lift Speed: ", liftSpeed);
        multiTelemetry.addData("Swing Position: ", swing.getPosition());
        multiTelemetry.addData("Swing Speed: ", swingIncrement);
        multiTelemetry.addData("Claw Pos: ", claw.getPosition());
    }

    public void handleDrive(){
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
        double scalar = 1 - inputHandler.rightTrigger.getValue();

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
                ).times(scalar)
        );
    }

    public void handleLift(){
        //Lift Control
        if (inputHandler2.buttonY.held) liftPos = (int) liftSpeed;
        if (inputHandler2.buttonA.held) liftPos = (int) -liftSpeed;
        //lift.goToPos(liftPos);
        //lift.setPower(liftPos);
        liftMotor.setPower(liftPos);

        if(!inputHandler2.buttonA.getState() || !inputHandler.buttonY.getState()) liftPos = 0;
        //lift.update();
    }

    public void handleHooks(){
        /*
        //Hook Control



        if (isHookDown) hookLeft.setPosition(hookDown);
        if (!isHookDown) hookLeft.setPosition(hookUp);

        hookRight.setPosition(1- hookOffset -hookLeft.getPosition());

         */
        if (inputHandler.buttonA.pressed) isHookDown = !isHookDown;
        double[] hookPosititon = hookController.updatePosition(isHookDown);

        hookLeft.setPosition(hookPosititon[0]);
        hookRight.setPosition(hookPosititon[1]);
    }

    public void handleIntake() {
        if (inputHandler.buttonB.pressed) isIntaking = !isIntaking;
        if (isIntaking) IntakeLeft.setPower(1);
        if (!isIntaking) IntakeLeft.setPower(0);
        IntakeRight.setPower(IntakeLeft.getPower()*-1);
    }

    public void handleClaw() {
        if(inputHandler2.rightBumper.released) {
            clawBool = !clawBool;
        }
        if(clawBool) {claw.setPosition(clawOpen);}
        else {claw.setPosition(clawClose);}
    }
    public void handleSwing() {
        double currentPos = swing.getPosition();
        double velocity = inputHandler2.rightTrigger.getValue() - inputHandler2.leftTrigger.getValue();
        if (inputHandler2.leftStickButton.pressed){
            swingIgnoreLimits = !swingIgnoreLimits;
        }
        double output;
        if (!swingIgnoreLimits) {
            output = Math.max(Math.min(currentPos + velocity * swingIncrement, upperLimit), lowerLimit);
        } else {
            output = Math.max(Math.min(currentPos + velocity * swingIncrement, 0.65), 0.3);
        }
        swing.setPosition(output);
    }
}
