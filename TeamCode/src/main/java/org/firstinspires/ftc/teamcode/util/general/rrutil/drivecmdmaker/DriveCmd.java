package org.firstinspires.ftc.teamcode.util.general.rrutil.drivecmdmaker;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.statemachine.State;

public class DriveCmd implements State {
    protected static SampleMecanumDrive drive;
    private Trajectory trajectory;

    public DriveCmd(Trajectory trajectory){
        this.trajectory = trajectory;
    }

    @Override
    public void init() {
        drive.setPoseEstimate(drive.getLastError());
        drive.followTrajectoryAsync(trajectory);
    }

    @Override
    public void run() {
        drive.update();
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return !(drive.isBusy() && !Thread.currentThread().isInterrupted());
    }
}
