package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;


@Autonomous(name = "Red_Goal")
public class Red_Goal extends OpMode {
    public Intake intake;
    public Shooter shooter = new Shooter(hardwareMap, telemetry);
    public Spindex spindex = new Spindex(hardwareMap, telemetry, shooter);


    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(122, 125, Math.toRadians(36));
    private final Pose aimPose = new Pose(104, 95, Math.toRadians(45));
    private final Pose clearPose = new Pose(94, 120, Math.toRadians(45));

    private PathChain move1, move2;

    public void buildPaths(){
        move1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, aimPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), aimPose.getHeading())
                .build();

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
        spindex.goToPos(spindex.targetPos, false);

        switch (pathState){
            case 0:
                shooter.primeShooter(3400);
                follower.followPath(move1, true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    spindex.shootSpindex(3400);
                    if(spindex.targetPos == Spindex.Offset.STORE1){
                        setPathState(2);
                    }
                }
                break;
            case 2:
                shooter.primeShooter(0);
                if(!follower.isBusy()){
                    follower.followPath(move2);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    setPathState(-1);
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
        telemetry.addData("Is busy", follower.isBusy());
        telemetry.addLine();
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("At RPM", shooter.isAtRPM(4500));
        telemetry.addData("At RPM", shooter.getRPM() > 4500);
        telemetry.addData("Spindex POS", spindex.targetPos);
        telemetry.addData("Spindex State", spindex.spindexState);
        telemetry.update();

        if (pathState == -1){
            requestOpModeStop();
        }
    }

    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap, "flm", "frm", "blm", "brm");
        buildPaths();
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        spindex = new Spindex(hardwareMap, telemetry, shooter);

        spindex.init("spinMotor", "transferServo", "magneticSwitch", "frontDistanceSensor", "backDistanceSensor", true);
        shooter.init("shooterMotor", "hoodServo");


        spindex.homeSpindex();
        spindex.saveHome();
        spindex.targetPos = Spindex.Offset.SHOOT1;
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


