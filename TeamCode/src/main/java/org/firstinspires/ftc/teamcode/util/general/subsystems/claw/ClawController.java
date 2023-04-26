package org.firstinspires.ftc.teamcode.util.general.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class ClawController {
    private Servo clawServo;
    private Servo swingServo;
    private boolean busy = false;

    private int CLAW_OPEN = 0; //PLACEHOLDER VALUE FOR CLAW OPEN POSITION
    private int CLAW_CLOSED = 0; //PLACEHOLDER VALUE FOR CLAW CLOSED POSITION

    private int SWING_MIN = 0; //PLACEHOLDER VALUE FOR THE MINIMUM VALUE OF THE SWING SERVO

    private int SWING_MAX = 0; //PLACEHOLDER VALUE FOR THE MAXIMUM VALUE OF THE SWING SERVO

    private double swingPos = 0.5;

    private boolean open = false;

    public ClawController(HardwareMap hardwareMap, String clawServoName, String swingServoName){
        clawServo = hardwareMap.servo.get(clawServoName);
        swingServo = hardwareMap.servo.get(swingServoName);
    }

    public boolean isBusy(){
        return busy;
    }

    public double getSwingPos() { return swingPos; }

    public void setSwingPos(double newSwingPos) { swingPos = newSwingPos; }

    public void setBusy(boolean nBool){
        this.busy = nBool;
    }

    public void toggleOpen() { open = !open; }

    public void setOpen(boolean state){
        open = state;
    }

    /**
     * This method is always meant to be called within the OpMode to make sure the lift stays on the target
     */
    public void update(){
        clawServo.setPosition( open ? CLAW_OPEN : CLAW_CLOSED );
        swingServo.setPosition(swingPos * (SWING_MAX-SWING_MIN) + SWING_MIN);
    }
}
