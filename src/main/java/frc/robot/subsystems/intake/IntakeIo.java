package frc.robot.subsystems.intake;


import org.littletonrobotics.junction.AutoLog;

public interface IntakeIo {
  @AutoLog
    public static class IntakeIoInputs{

    }
    public default void updateInputs(IntakeIoInputs inputs){

    }
    public default void setMotor(double speed){

    }
    public default void resetIntakeEncoder(){
        
    }
}
