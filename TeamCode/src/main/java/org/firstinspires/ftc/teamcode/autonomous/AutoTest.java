package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;


@Autonomous(name = "AutoTest")
public class AutoTest extends OpMode {

    Spindex spindex = new Spindex(hardwareMap, telemetry);
    Intake intake = new Intake(hardwareMap, telemetry);
    Shooter shooter = new Shooter(hardwareMap, telemetry);

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose aimPose = new Pose(12, 12, Math.toRadians(90));
    private final Pose intake1Lineup = new Pose(0, 0, Math.toRadians(180));
    private final Pose intake1Intake = new Pose(0, 0, Math.toRadians(0));


    private Path move1;
    private PathChain move2, move3;

    public void buildPaths(){
        move1 = new Path(new BezierLine(startPose, aimPose));
        move1.setLinearHeadingInterpolation(startPose.getHeading(), intake1Lineup.getHeading());

        move2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Lineup, intake1Intake))
                .setLinearHeadingInterpolation(intake1Lineup.getHeading(), intake1Intake.getHeading())
                .build();

        move3 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Intake, aimPose))
                .setLinearHeadingInterpolation(intake1Lineup.getHeading(), aimPose.getHeading())
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
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(move3);
                    setPathState(3);
                }
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


