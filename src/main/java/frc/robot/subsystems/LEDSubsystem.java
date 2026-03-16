package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANBus kCanBus = new CANBus("Herbivore");
    private final CANdle m_candle = new CANdle(25, kCanBus);

    private static final RGBWColor redColor = new RGBWColor(255, 0, 0,0);

    private final RainbowAnimation mRainbowAnimation = new RainbowAnimation(0, 7);
    private final FireAnimation mFireAnimation = new FireAnimation(0, 7);

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        m_candle.getConfigurator().apply(config);
    }

    public void setLights() {
        //m_candle.setControl(new SolidColor(0, 7).withColor(redColor));
        //m_candle.setControl(mRainbowAnimation);
        m_candle.setControl(mFireAnimation);
    }

}
