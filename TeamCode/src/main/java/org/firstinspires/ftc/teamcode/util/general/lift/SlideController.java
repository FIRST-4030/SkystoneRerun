package org.firstinspires.ftc.teamcode.util.general.lift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideController {


    private DcMotor right;

    public int target;

    public int rightEncoderPosition = 0;
    public double powerOutput = 0;

    private boolean inUse = false;

    public SlideController(HardwareMap hardwareMap, String rightMotorName, boolean invertRight){
        right = hardwareMap.dcMotor.get(rightMotorName);

        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setTargetPosition(0);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setDirection(invertRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public SlideController(HardwareMap map){}

    public void update(double deltaTime, double slidePower){

        rightEncoderPosition = right.getCurrentPosition();

        double p = 1;
        powerOutput = p * rightEncoderPosition;
    }



/*
    public void update(double deltaTime, LEVEL level, double slidePower){

        leftEncoderPosition = left.getCurrentPosition();
        rightEncoderPosition = right.getCurrentPosition();

        switch (level){
            case REST:
                left.setTargetPosition(0);
                right.setTargetPosition(0);
                break;
            case LOW:
                left.setTargetPosition(540 / 3 - 50);
                right.setTargetPosition(540 / 3 - 50);
                break;
            case MIDDLE:
                left.setTargetPosition(540 / 3 + 100);
                right.setTargetPosition(540 / 3 + 100);
                break;
            case HIGH:
                left.setTargetPosition(540 / 3 + 250);
                right.setTargetPosition(540 / 3 + 250);
                break;
        }

        if (Math.abs(rightLastEncoderPosition - right.getTargetPosition()) >= tickTolerance) {
            left.setPower(Math.abs(rightLastEncoderPosition - right.getTargetPosition())/50f);
            right.setPower(Math.abs(rightLastEncoderPosition - right.getTargetPosition())/50f);
        } else {
            left.setPower(0.5);
            left.setTargetPosition(leftEncoderPosition);
            right.setPower(0.5);
            right.setTargetPosition(rightEncoderPosition);
        }

        inUse = !(level == LEVEL.REST);

        leftLastEncoderPosition = leftEncoderPosition;
        rightLastEncoderPosition = rightEncoderPosition;

    }

 */


    public void setPower(double power){
        right.setPower(power);
    }

    public void logMotorPos(Telemetry telemetry){
        telemetry.addData("LSRM Encoder Position: ", right.getCurrentPosition());
    }

    public boolean isInUse(){
        return inUse;
    }

    public DcMotor getRight(){
        return right;
    }
}