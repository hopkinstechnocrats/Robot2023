package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NEOTestSubsystem extends SubsystemBase {
    private final CANSparkMax testNEO = new CANSparkMax(0, MotorType.kBrushless);

    public void SpinNEO(double speed) {
        testNEO.set(speed);
    }
    
}
