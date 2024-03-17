package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.clawFlipController.clawFlipStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.clawFlipController.clawFlipStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawFlipController.clawFlipStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawFlipController.clawFlipStatus.PURPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawFlipController.clawFlipStatus.SCORE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class clawFlipController {

    public enum clawFlipStatus {
        INITIALIZE,
        COLLECT,
        SCORE,
        DRIVE,
        PURPLE
    }

    public clawFlipController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static clawFlipStatus CS = INITIALIZE, PS = INITIALIZE;

    public static  double init = 0.95;
    public static double collect = 0.78;
    public static double score = 0.3;
    public static double drive = 0.95;
    public static double purple = 0.15;

    public void update(robotMap r) {

        if (CS != PS || CS == INITIALIZE || CS == COLLECT || CS == SCORE || CS == DRIVE || CS == PURPLE) {

            switch (CS) {

                case INITIALIZE: {
                    r.clawFlip.setPosition(init);
                    break;
                }

                case COLLECT: {
                    r.clawFlip.setPosition(collect);
                    break;
                }

                case SCORE: {
                    r.clawFlip.setPosition(score);
                    break;
                }

                case DRIVE:
                {
                    r.clawFlip.setPosition(drive);
                    break;
                }
                case PURPLE:
                {
                    r.clawFlip.setPosition(purple);
                    break;
                }

            }
        }

        PS = CS;
    }

}
