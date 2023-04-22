package org.firstinspires.ftc.teamcode.util.general.lift;

import org.firstinspires.ftc.teamcode.util.statemachine.State;

public class LiftCmd implements State {

    private LiftController liftController;
    private int target;

    public LiftCmd(LiftController lift, int encoderTarget){
        this.liftController = lift;
        this.target = encoderTarget;
    }

    @Override
    public void init() {
        this.liftController.setBusy(true);
        this.liftController.pidfController.setTargetPosition(target);
    }

    @Override
    public void run() {}

    @Override
    public void end() {
        liftController.setBusy(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
