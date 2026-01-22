package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;
import frc.team3602.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase{
    //variables
    private static TalonFX intakeMotor;
    
    //constructor
    public IntakeSubsystem(){
        intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID);
    }

    public Command setIntakeSpeed(double speed) {
        return runOnce(() -> 
            intakeMotor.set(speed));
    
    }
}

