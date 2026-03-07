package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Intake;
import org.firstinspires.ftc.teamcode.utils.Shooter;
import org.firstinspires.ftc.teamcode.utils.Spindex;


@Autonomous(name = "Red_Goal")
public class Red_Goal extends OpMode {
    private Follower follower;
    public Intake intake;
    public Shooter shooter = new Shooter(hardwareMap, telemetry);
    public Spindex spindex = new Spindex(hardwareMap, telemetry, shooter);
    public DriveTrain driveTrain = new DriveTrain(hardwareMap, telemetry, follower);


    private Timer pathTimer, opmodeTimer;
    private int pathState;


    private final Pose startPose = new Pose(122, 125, Math.toRadians(36));
    private final Pose aimPose = new Pose(96, 95, Math.toRadians(40));
    private final Pose intakePose1 = new Pose(98, 80, Math.toRadians(0));
    private final Pose collectPose1 = new Pose(122, 80, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(98, 56, Math.toRadians(0));
    private final Pose collectPose2 = new Pose(122, 56, Math.toRadians(0));
    private final Pose intakePose3 = new Pose(96, 32, Math.toRadians(0));
    private final Pose collectPose3 = new Pose(122, 32, Math.toRadians(0));
    private final Pose clearPose = new Pose(94, 120, Math.toRadians(0));


    //    private Path move1;
    private PathChain gotoShoot1, gotoIntake1, gotoCollect1, gotoShoot2, gotoIntake2, gotoCollect2, gotoShoot3, gotoIntake3, gotoCollect3, gotoShoot4, gotoClear;

    public void buildPaths(){
        gotoShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, aimPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), aimPose.getHeading())
                .build();

        gotoIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(aimPose, intakePose1))
                .setLinearHeadingInterpolation(aimPose.getHeading(), clearPose.getHeading())
                .build();

        gotoCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, collectPose1))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), collectPose1.getHeading())
                .build();

        gotoShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose1, aimPose))
                .setLinearHeadingInterpolation(collectPose1.getHeading(), aimPose.getHeading())
                .build();

        gotoIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(aimPose, intakePose2))
                .setLinearHeadingInterpolation(aimPose.getHeading(), intakePose2.getHeading())
                .build();

        gotoCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, collectPose2))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), collectPose2.getHeading())
                .build();

        gotoShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose2, aimPose))
                .setLinearHeadingInterpolation(collectPose2.getHeading(), aimPose.getHeading())
                .build();

        gotoIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(aimPose, intakePose3))
                .setLinearHeadingInterpolation(aimPose.getHeading(), intakePose3.getHeading())
                .build();

        gotoCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3, collectPose3))
                .setLinearHeadingInterpolation(intakePose3.getHeading(), collectPose3.getHeading())
                .build();

        gotoShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(collectPose3, aimPose))
                .setLinearHeadingInterpolation(collectPose3.getHeading(), aimPose.getHeading())
                .build();

        gotoClear = follower.pathBuilder()
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
                shooter.primeShooter(2800);
                follower.followPath(gotoShoot1);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    spindex.shootSpindex(2800, true);
                    if(spindex.targetPos == Spindex.Offset.STORE1){
                        shooter.primeShooter(0);
                        follower.followPath(gotoIntake1);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    intake.forwardIntake();
                    follower.followPath(gotoCollect1, 0.25, false);
                    setPathState(3);
                }
                break;
            case 3:
                spindex.intakeSpindex();
                if(!follower.isBusy() || spindex.targetPos == Spindex.Offset.SHOOT1){
                    intake.stopIntake();
                    spindex.targetPos = Spindex.Offset.SHOOT1;
                    shooter.primeShooter(2800);
                    follower.followPath(gotoShoot2);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    spindex.shootSpindex(2800, true);
                    if(spindex.targetPos == Spindex.Offset.STORE1){
                        shooter.primeShooter(0);
                        follower.followPath(gotoIntake2);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    intake.forwardIntake();
                    follower.followPath(gotoCollect2, 0.25, false);
                    setPathState(6);
                }
                break;
            case 6:
                spindex.intakeSpindex();
                if(!follower.isBusy() || spindex.targetPos == Spindex.Offset.SHOOT1){
                    intake.stopIntake();
                    spindex.targetPos = Spindex.Offset.SHOOT1;
                    shooter.primeShooter(2800);
                    follower.followPath(gotoShoot3);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    spindex.shootSpindex(2800, true);
                    if(spindex.targetPos == Spindex.Offset.STORE1){
                        shooter.primeShooter(0);
                        follower.followPath(gotoIntake3);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    intake.forwardIntake();
                    follower.followPath(gotoCollect3, 0.25, false);
                    setPathState(9);
                }
                break;
            case 9:
                spindex.intakeSpindex();
                if(!follower.isBusy() || spindex.targetPos == Spindex.Offset.SHOOT1){
                    intake.stopIntake();
                    spindex.targetPos = Spindex.Offset.SHOOT1;
                    shooter.primeShooter(2800);
                    follower.followPath(gotoShoot4);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    spindex.shootSpindex(2800, true);
                    if(spindex.targetPos == Spindex.Offset.STORE1){
                        shooter.primeShooter(0);
                        follower.followPath(gotoClear, 1, true);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    stop();
                }
                break;
        }
    }


    @Override
    public void loop(){
        follower.update();
        spindex.updateKicker();
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

        follower = Constants.createFollower(hardwareMap, "fl", "fr", "bl", "br");
        buildPaths();
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        spindex = new Spindex(hardwareMap, telemetry, shooter);

        intake.init("intakeMotor");
        spindex.init("spinMotor", "kickServo", "magneticSwitch", "distanceSensor", "transferMotor", true);
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

    @Override
    public void stop(){
        driveTrain.savePose(follower);
    }
}


