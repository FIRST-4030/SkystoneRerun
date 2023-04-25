package org.firstinspires.ftc.teamcode.util.general;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.general.lift.LiftCmd;


@Config
public class ClawController {
    private Servo clawServo;
    private Servo swingServo;
    private boolean busy = false;

    private int CLAW_OPEN = 0; //PLACEHOLDER VALUE FOR CLAW OPEN POSITION
    private int CLAW_CLOSED = 0; //PLACEHOLDER VALUE FOR CLAW CLOSED POSITION

    private boolean open = false;

    public ClawController(HardwareMap hardwareMap, String clawServoName, String swingServoName){
        clawServo = hardwareMap.servo.get(clawServoName);
        swingServo = hardwareMap.servo.get(swingServoName);
    }

    public boolean isBusy(){
        return busy;
    }

    public void setBusy(boolean nBool){
        this.busy = nBool;
    }

    public void toggleOpen() { open = !open; }


    /**
     * This method is always meant to be called within the OpMode to make sure the lift stays on the target
     */
    public void update(){ clawServo.setPosition( open ? CLAW_OPEN : CLAW_CLOSED ); }
}
