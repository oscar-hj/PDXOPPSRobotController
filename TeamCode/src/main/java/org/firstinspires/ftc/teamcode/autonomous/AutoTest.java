package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose midPose = new Pose(12, 12, Math.toRadians(90));
    private final Pose endPose = new Pose(0, 0, Math.toRadians(180));


    private Path move1;
    private PathChain move2;

    public void buildPaths(){
        move1 = new Path(new BezierLine(startPose, midPose));
        move1.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        move2 = follower.pathBuilder()
                .addPath(new BezierLine(midPose, endPose))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(move1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(move2);
                    setPathState(2);
                }
                break;
        }
    }


    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Program: Auto Test");
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}


