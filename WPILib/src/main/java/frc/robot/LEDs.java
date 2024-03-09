package frc.robot;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;

public class LEDs
{
    private static AddressableLED LED;
    private static AddressableLEDBuffer ColorBuffer;

    public static void Setup()
    {
        LED = new AddressableLED(0);
        ColorBuffer = new AddressableLEDBuffer(60);
        LED.setLength(ColorBuffer.getLength());
        LED.setData(ColorBuffer);
        LED.start();
        SetAll(255, 255, 255);

        System.out.println("Setup LEDs!");
    }

    public static void SetAll(int h, int s, int v)
    {
        if (RobotBase.isSimulation()) return;

        for (var i = 0; i < ColorBuffer.getLength(); i++)
        {
            ColorBuffer.setHSV(i, h, s, v);
        }
        Write();
    }
    
    public static void SetRange(int start, int end, int h, int s, int v)
    {
        if (RobotBase.isSimulation()) return;

        for (var i = start; i < end; i++)
        {
            ColorBuffer.setHSV(i, h, s, v);
        }
        Write();
    }

    public static void Write()
    {
        LED.setData(ColorBuffer);
        System.out.println("Updated LEDs!");
    }

    private static int RainbowHueStep = 0;
    public static void Rainbow()
    {
        // For every pixel
        for (var i = 0; i < ColorBuffer.getLength(); i++)
        {
            // shape is a circle so only one value needs to precess
            final int hue = (RainbowHueStep + (i * 180 / ColorBuffer.getLength())) % 180;

            // Set the value
            ColorBuffer.setHSV(i, hue, 255, 128);
        }

        // Increase by to make the rainbow "move"
        RainbowHueStep += 3;

        // Check bounds
        RainbowHueStep %= 180;

        Write();
    }

    public static void SetArm(int r, int g, int b)
    {
        SetRange(0, 288, r, g, b);
    }
}
