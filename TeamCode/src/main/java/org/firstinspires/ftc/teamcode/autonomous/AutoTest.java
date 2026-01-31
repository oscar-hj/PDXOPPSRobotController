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
    public Intake intake;
    public Shooter shooter = new Shooter(hardwareMap, telemetry);
    public Spindex spindex = new Spindex(hardwareMap, telemetry, shooter);


    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(0, 0, Math.toRadians(270));
    private final Pose aimPose = new Pose(24, 24, Math.toRadians(135));
    private final Pose clearPose = new Pose(0, 0, Math.toRadians(0));


    private Path move1;
    private PathChain move2, move3;

    public void buildPaths(){
        move1 = new Path(new BezierLine(startPose, aimPose));
        move1.setLinearHeadingInterpolation(startPose.getHeading(), aimPose.getHeading());

        move2 = follower.pathBuilder()
                .addPath(new BezierLine(aimPose, clearPose))
                .setLinearHeadingInterpolation(aimPose.getHeading(), clearPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate(){
        spindex.goToPos(spindex.currentPos);

        switch (pathState){
            case 0:
                shooter.primeShooter(6000);
                follower.followPath(move1);
                if(!follower.isBusy()){
                    spindex.shootSpindex();
                }
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

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        spindex = new Spindex(hardwareMap, telemetry, shooter);

        spindex.init("spinMotor", "transferServo", "magneticSwitch", "frontDistanceSensor", "backDistanceSensor", true);
        shooter.init("shooterMotor", "hoodServo");


        spindex.homeSpindex();
        spindex.saveHome();
        spindex.currentPos = Spindex.Offset.SHOOT1;
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


