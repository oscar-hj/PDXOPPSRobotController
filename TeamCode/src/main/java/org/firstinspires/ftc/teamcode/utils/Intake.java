package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotor intakeMotor;
    TouchSensor magneticSwitch;
    public IntakeState state;

    public enum IntakeState{
        INTAKE_ON,
        INTAKE_OFF,
        INTAKE_BACKWARDS
    }

    public Intake(HardwareMap hwm, Telemetry tel){
        this.hardwareMap = hwm;
        this.telemetry = tel;
    }

    public void init(String motorName){
        intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void forwardIntake(){
        intakeMotor.setPower(1);
        telemetry.addData("Whatever", intakeMotor.getPower());
        state = IntakeState.INTAKE_ON;
    }

    public void backwardsIntake(){
        intakeMotor.setPower(-1);
        telemetry.addData("Whatever", intakeMotor.getPower());

        state = IntakeState.INTAKE_BACKWARDS;
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
        telemetry.addData("Whatever", intakeMotor.getPower());
        state = IntakeState.INTAKE_OFF;
    }

}
