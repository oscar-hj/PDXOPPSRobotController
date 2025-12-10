package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GP {
    public Gamepad gamepad;
    public double LSX, LSY, RSX, RSY, LT, RT;
    public boolean X, Y, A, B, LB, RB, PS, DPU, DPD, DPL, DPR;

    public GP(Gamepad gp){
        this.gamepad = gp;
    }
    public void readGP(){
        LSX = gamepad.left_stick_x;
        LSY = gamepad.left_stick_y;
        RSX = gamepad.right_stick_x;
        RSY = gamepad.right_stick_y;

        LT  = gamepad.left_trigger;
        RT  = gamepad.right_trigger;

        X = gamepad.x;
        Y = gamepad.y;
        A = gamepad.a;
        B = gamepad.b;

        LB = gamepad.left_bumper;
        RB = gamepad.right_bumper;
        PS = gamepad.ps;

        DPU = gamepad.dpad_up;
        DPD = gamepad.dpad_down;
        DPL = gamepad.dpad_left;
        DPR = gamepad.dpad_right;
    }
}
