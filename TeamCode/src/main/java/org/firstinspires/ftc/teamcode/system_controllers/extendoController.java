package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.INITIALIZE;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.globals.robotMap;

@Config
public class extendoController {

    public enum extendoStatus
    {
        INITIALIZE,
        RETRACTED,
        EXTENDED,
        AUTO,
    }

    public static double Kp = 0.0065;
    public static double Ki = 0.0043;
    public static double Kd = 0.001;

    public static double maxSpeed = 1;

    public extendoStatus CS = INITIALIZE, PS = INITIALIZE;

    SimplePIDController extendoPID = null;

    public static double CurrentPosition = 0;
    public static double retracted = -7;
    public static double extended = 900;
    public static double auto[] = {420, 375, 375, 375, 375};
    public int ciclu = 0;

    public static double extend_multiply_index = 0;

      public extendoController()
      {
          extendoPID = new SimplePIDController(Kp, Ki, Kd);
          extendoPID.targetValue = retracted;
          extendoPID.maxOutput = maxSpeed;
      }

      public void update(robotMap r, int position, double powerCap, double voltage)
      {

          if(CS == EXTENDED)
          {
              extendoPID.targetValue = extended + extend_multiply_index;
          }

          CurrentPosition = position;
          double powerColectare = extendoPID.update(position);
          powerColectare = Math.max(-powerCap,Math.min(powerColectare* 14 / voltage,powerCap));
         r.extendoLeft.setPower(powerColectare);
          r.extendoRight.setPower(powerColectare);

         if(CS != PS || CS == INITIALIZE)
         {
             switch (CS)
             {
                 case INITIALIZE:
                 {
                     extendoPID.targetValue = retracted;
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case EXTENDED:
                 {
                     extendoPID.targetValue = extended + extend_multiply_index;
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case RETRACTED:
                 {
                     extendoPID.targetValue = retracted;
                     extendoPID.maxOutput = 1;
                     break;
                 }

                 case AUTO:
                 {
                     extendoPID.targetValue = auto[ciclu];
                     extendoPID.maxOutput = 1;
                     break;
                 }
             }

             PS = CS;
         }

      }

}
