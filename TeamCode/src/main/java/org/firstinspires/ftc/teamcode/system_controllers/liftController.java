package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class liftController {

    public enum liftStatus
    {
        INITIALIZE,
        DOWN,
        UP,
    }

    public double CurrentSpeed = 0;

    /**
     * UP
     */

    public static double P1 = 0.027;
    public static double I1 = 0;
    public static double D1 = 0;

    public double Kg = 0;
    public double maxSpeedUp = 1;

    public static liftStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController LiftPID = null;


    int base = 0;

    public static double down = 90;

    public static int i_up = 0;
    public static double i_multiplication = 90;

    public int CurrentPosition = 0;

    public liftController()
    {
        LiftPID = new SimplePIDController(P1,I1,D1);
        LiftPID.targetValue = base;
        LiftPID.maxOutput = maxSpeedUp;
    }

    public void update(robotMap r, int position, double voltage)
    {

        CurrentPosition = position;

        if(CS == liftStatus.UP) LiftPID.targetValue = down + i_up * i_multiplication;



        double powerLift = LiftPID.update(position) + Kg;
        powerLift = Math.max(-1,Math.min(powerLift* 14 / voltage,1));
        CurrentSpeed=powerLift;

        r.lift.setPower(powerLift);

        if(CS != PS || CS == INITIALIZE )
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    LiftPID.targetValue = 0;
                    break;
                }

                case UP:
                {
                    LiftPID.targetValue = down + i_up * i_multiplication;
                    break;
                }

                case DOWN:
                {
                    LiftPID.targetValue = -20;
                    break;
                }
            }
        }

        PS = CS;
    }

}
