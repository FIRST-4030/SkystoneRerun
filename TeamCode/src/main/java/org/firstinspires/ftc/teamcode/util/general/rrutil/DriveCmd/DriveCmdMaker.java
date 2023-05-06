package org.firstinspires.ftc.teamcode.util.general.rrutil.DriveCmd;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.statemachine.State;

public class DriveCmdMaker {
    private static DriveCmdMaker instance = null;
    private State.Sequence driveSequence;

    private DriveCmdMaker(){
        driveSequence = new State.Sequence();
    }

    public static void init(SampleMecanumDrive drive){
        if (instance == null) {
            DriveCmd.drive = drive;
            instance = new DriveCmdMaker();
        }
    }

    public static DriveCmdMaker getInstance(){
        return instance;
    }

    public DriveCmdMaker addDriveCmd(Trajectory trajectory){
        driveSequence.add(new DriveCmd(trajectory));
        return this;
    }

    public DriveCmdMaker addMultiDriveCmd(Trajectory... trajectories){
        for (Trajectory t: trajectories) {
            driveSequence.add(new DriveCmd(t));
        }
        return this;
    }

    public DriveCmdMaker addIntermediateState(State intermediateState){
        driveSequence.add(intermediateState);
        return this;
    }

    public DriveCmdMaker addConcurrentState(Trajectory trajectory, State... states){
        State[] driveAndStates = new State[states.length + 1];
        driveAndStates[0] = new DriveCmd(trajectory);
        System.arraycopy(states, 0, driveAndStates, 1, driveAndStates.length - 1);
        driveSequence.add(new State.AsyncGroup(driveAndStates));
        return this;
    }

    public State.Sequence build(){
        return driveSequence;
    }
}
