package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Position;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class SetPosition extends SequentialCommandGroup {

    public SetPosition(Wrist s_Wrist, Elevator s_Elevator, Position position, Supplier<GamePiece> gamePiece) {

        // Defaults to Stowed Position
        Command setElevatorPose = s_Elevator.setPose(0.25);
        Command setWristPose = s_Wrist.setPose(1.1765);
        boolean sequential = false;

        switch (position) {
            case HIGH:
                if (gamePiece.get() == GamePiece.CONE) {
                    setWristPose = s_Wrist.setPose(.104327); 
                    setElevatorPose = s_Elevator.setPose(35);
                    sequential = true;                       
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    setWristPose = s_Wrist.setPose(1.55);
                    setElevatorPose = s_Elevator.setPose(35.1);
                }
                break;

            case MID:
                if (gamePiece.get() == GamePiece.CONE) {
                    setWristPose = s_Wrist.setPose(.104327);
                    setElevatorPose = s_Elevator.setPose(23);
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    setWristPose = s_Wrist.setPose(1.427);
                    setElevatorPose = s_Elevator.setPose(16.5);
                }
                break;

            case LOW:
                setWristPose = s_Wrist.setPose(0.5236);
                setElevatorPose = s_Elevator.setPose(0.25);
                break;

            case STANDBY:
                setWristPose = s_Wrist.setPose(1.1765);
                setElevatorPose = s_Elevator.setPose(0.25);
                break;

            case CUBEINTAKE:
                setWristPose = s_Wrist.setPose(0);
                setElevatorPose = s_Elevator.setPose(0.25);
                break;

            case STANDINGCONEINTAKE:
                setWristPose = s_Wrist.setPose(5.106);
                setElevatorPose = s_Elevator.setPose(14.380);
                break;

            case TIPPEDCONEINTAKE:
                setWristPose = s_Wrist.setPose(5.572);
                setElevatorPose = s_Elevator.setPose(1.333);
                break;

            case HUMANPLAYERINTAKE:
                setWristPose = s_Wrist.setPose(.8763);
                setElevatorPose = s_Elevator.setPose(3.5472);
                break;
        }

        if(sequential) {
            addCommands(setElevatorPose, setWristPose);
        }
        else {
            addCommands(new ParallelCommandGroup(setElevatorPose, setWristPose));
        }
    }

}
