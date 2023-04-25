package org.firstinspires.ftc.teamcode.util.general.subsystems.lift;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class LiftController {
    private DcMotor motor;
    private boolean busy = false;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients();
    protected PIDFController pidfController;
    public LiftController(HardwareMap hardwareMap, String motorName, boolean invertRight){
        motor = hardwareMap.dcMotor.get(motorName);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(invertRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidfController = new PIDFController(pidCoefficients);
        pidfController.setOutputBounds(-1, 1);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void setTargetPosition(int position){
        pidfController.setTargetPosition(position);
    }

    public int getMotorPosition(){
        return motor.getCurrentPosition();
    }

    public boolean isBusy(){
        return busy;
    }

    public void setBusy(boolean nBool){
        this.busy = nBool;
    }

    /**
     * create a lift target that is compatible with the state sequence/machine
     * @param target
     * @return LiftCmd with a target encoder position
     */
    public LiftCmd createTargetState(int target){
        return new LiftCmd(this, target);
    }

    /**
     * This method is always meant to be called within the OpMode to make sure the lift stays on the target
     */
    public void update(){
        setPower(pidfController.update(motor.getCurrentPosition()));
    }
}