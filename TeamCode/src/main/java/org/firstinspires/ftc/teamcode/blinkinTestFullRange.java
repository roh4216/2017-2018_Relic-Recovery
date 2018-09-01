package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="blinkinTestFullRange", group="OpMode")
//@Disabled

public class blinkinTestFullRange extends OpMode {

    Servo blinkin;

    double color;

    @Override
    public void init() {

       blinkin = hardwareMap.servo.get("blinkin");
    }

    @Override
    public void start(){



    }

    @Override
    public void loop() {

        blinkin.setPosition(0.7075);

    }


    @Override
    public void stop() {

    }

    public static float[] RGBtoHSB(int red, int green, int blue, float array[])
 {
         if (array == null)
               array = new float[3];
         // Calculate brightness.
         int min;
         int max;
         if (red < green)
               {
                 min = red;
                 max = green;
               }
         else
           {
                 min = green;
                 max = red;
               }
         if (blue > max)
               max = blue;
         else if (blue < min)
               min = blue;
         array[2] = max / 255f;
         // Calculate saturation.
         if (max == 0)
               array[1] = 0;
         else
           array[1] = ((float) (max - min)) / ((float) max);
         // Calculate hue.
         if (array[1] == 0)
               array[0] = 0;
         else
           {
                 float delta = (max - min) * 6;
                 if (red == max)
                   array[0] = (green - blue) / delta;
                 else if (green == max)
                  array[0] = 1f / 3 + (blue - red) / delta;
                 else
                   array[0] = 2f / 3 + (red - green) / delta;
                 if (array[0] < 0)
                  array[0]++;
               }
         return array;
          }
    private static int convert(float red, float green, float blue, float alpha)
   {
             if (red < 0 || red > 1 || green < 0 || green > 1 || blue < 0 || blue > 1
                 || alpha < 0 || alpha > 1)
               throw new IllegalArgumentException("Bad RGB values");
             int redval = Math.round(255 * red);
             int greenval = Math.round(255 * green);
             int blueval = Math.round(255 * blue);
             int alphaval = Math.round(255 * alpha);
             return (alphaval << 24) | (redval << 16) | (greenval << 8) | blueval;
           }
    public static int HSBtoRGB(float hue, float saturation, float brightness)
    {
        if (saturation == 0)
              return convert(brightness, brightness, brightness, 0);
        if (saturation < 0 || saturation > 1 || brightness < 0 || brightness > 1)
              throw new IllegalArgumentException();
        hue = hue - (float) Math.floor(hue);
        int i = (int) (6 * hue);
        float f = 6 * hue - i;
        float p = brightness * (1 - saturation);
        float q = brightness * (1 - saturation * f);
        float t = brightness * (1 - saturation * (1 - f));
        switch (i)
          {
              case 0:
              return convert(brightness, t, p, 0);
        case 1:
              return convert(q, brightness, p, 0);
        case 2:
              return convert(p, brightness, t, 0);
        case 3:
              return convert(p, q, brightness, 0);
        case 4:
              return convert(t, p, brightness, 0);
        case 5:
              return convert(brightness, p, q, 0);
        default:
              throw new InternalError("impossible");
            }
         }

}


