package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ScoreGamePiece;

public class Elevator extends SubsystemBase {

    private final CANSparkMax elevatorMotorLeft; //making the left the lead motor
    private final CANSparkMax elevatorMotorRight; //the right motor is the follower



    
  //  private final CANCoder elevatorLeftController;
    //private final CANCoder elevatorRightController;

    private PIDController pidController;

    private double currentPosition;


    /**
     * Initialize Elevator motor and the built in encoder. There are no cancoders on the elevator
     */
    public Elevator() {
        //initialize motors
        //the right motor will spin clockwise and the left motor will go counter clockwise
        elevatorMotorLeft = new CANSparkMax(Constants.Elevator.motorLeftId, MotorType.kBrushless);

        elevatorMotorRight = new CANSparkMax(Constants.Elevator.motorRightId, MotorType.kBrushless);

        elevatorMotorLeft.restoreFactoryDefaults(); 
        elevatorMotorRight.restoreFactoryDefaults(); 

        elevatorMotorRight.setInverted(true);
        
        //elevatorRightController = new CANCoder(Constants.Elevator.canConderRightId);

        //The motors will follow each other
        //The right motor will follow whatever the applied output on the
        //left motor is so only need to adjust output for the left motor
        

        //initialize pidContoller
        pidController = new PIDController(Constants.Elevator.elevatorKP, Constants.Elevator.elevatorKI, Constants.Elevator.elevatorKD);
        pidController.setSetpoint(0);
        pidController.setTolerance(.1);

        
    }

    public void resetEncoder(){
        elevatorMotorLeft.getEncoder().setPosition(0);
        elevatorMotorRight.getEncoder().setPosition(0);
    }

    public void raise(double distance){
        updatePosition();
        distance -= currentPosition;
        this.move(MathUtil.clamp(pidController.calculate(distance), -Constants.Elevator.maxMotorSpeed, Constants.Elevator.maxMotorSpeed));
    }
    public void move(double speed){
        elevatorMotorRight.set(speed*Constants.Elevator.maxMotorSpeed);
        elevatorMotorLeft.set(speed*Constants.Elevator.maxMotorSpeed);
    }
    public boolean reachedSetpoint(double distance){
        return pidController.getPositionTolerance() <= Math.abs(currentPosition - distance);
    }

    private void updatePosition(){
       currentPosition = (-elevatorMotorLeft.getEncoder().getPosition() + elevatorMotorRight.getEncoder().getPosition())/2;
    }

    public void score(int level, boolean cone){
        new ScoreGamePiece(this, level, cone);
    }

    public void periodic(){
        SmartDashboard.putNumber("Left Elevator Motor", -elevatorMotorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Elevator Motor", elevatorMotorRight.getEncoder().getPosition());
    }
}