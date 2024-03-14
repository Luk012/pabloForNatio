package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class fourbarController {

    public enum fourbarStatus {
        SCORE,
        COLLECT,
        INITIALIZE,
        DRIVE,
        INTER,
        NEUTRAL,
        PRELOAD,
    }

    public fourbarController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static fourbarStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double score = 0.18;
    public static double collect = 0.92;
    public static double drive = 0.8;
    public static double preload = 0.075;
    public static double neutral = 0.5;

    public void update(robotMap r) {

        if (PS != CS || CS == INITIALIZE) {

            switch (CS) {

                case INITIALIZE: {
                    r.fourbarLeft.setPosition(drive);
                    r.fourbarRight.setPosition(drive);
                    break;
                }

                case COLLECT: {
                    r.fourbarLeft.setPosition(collect);
                    r.fourbarRight.setPosition(collect);
                    break;
                }

                case SCORE: {
                    r.fourbarLeft.setPosition(score);
                    r.fourbarRight.setPosition(score);
                    break;
                }

                case DRIVE:
                {
                    r.fourbarLeft.setPosition(drive);
                    r.fourbarRight.setPosition(drive);
                    break;
                }

                case NEUTRAL:
                {
                    r.fourbarLeft.setPosition(neutral);
                    r.fourbarRight.setPosition(neutral);
                    break;
                }

                case PRELOAD:
                {
                    r.fourbarLeft.setPosition(preload);
                    r.fourbarRight.setPosition(preload);
                    break;
                }

            }
            PS = CS;
        }
    }

}
