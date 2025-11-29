package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotor intakeMotor;
    public IntakeState state;

    public enum IntakeState{
        INTAKE_ON,
        INTAKE_OFF,
        INTAKE_BACKWARDS
    }
    public enum SwitchState{
        ACTIVATE,
        DEACTIVATE,
        ACTIVATE_BACKWARDS
    }

    public Intake(HardwareMap hwm, Telemetry tel){
        this.hardwareMap = hwm;
        this.telemetry = tel;
    }

    public void init(String motorName){
        intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void changeIntakeState(SwitchState intakeState){
        switch (intakeState){
            case ACTIVATE:
                intakeMotor.setPower(1);
                state = IntakeState.INTAKE_ON;
                telemetry.addData("Intake Mode", "On");
                break;
            case DEACTIVATE:
                intakeMotor.setPower(0);
                state = IntakeState.INTAKE_OFF;
                telemetry.addData("Intake Mode", "Off");
                break;
            case ACTIVATE_BACKWARDS:
                intakeMotor.setPower(-1);
                state = IntakeState.INTAKE_BACKWARDS;
                telemetry.addData("Intake Mode", "Backwards");
                break;
        }

    }
}
