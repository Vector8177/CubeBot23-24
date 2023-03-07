package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
                    s_Wrist.setPose(.104327);
                    s_Elevator.setPose(23);
                } else if (gamePiece.get() == GamePiece.CUBE) {
                    s_Wrist.setPose(1.427);
                    s_Elevator.setPose(16.5);
                }
                break;

            case LOW:
                s_Wrist.setPose(0.5236);
                s_Elevator.setPose(0.25);
                break;

            case STANDBY:
                s_Wrist.setPose(1.1765);
                s_Elevator.setPose(0.25);
                break;

            case CUBEINTAKE:
                s_Wrist.setPose(0);
                s_Elevator.setPose(0.25);
                break;

            case STANDINGCONEINTAKE:
                s_Wrist.setPose(5.106);
                s_Elevator.setPose(14.380);
                break;

            case TIPPEDCONEINTAKE:
                s_Wrist.setPose(5.572);
                s_Elevator.setPose(1.333);
                break;

            case HUMANPLAYERINTAKE:
                s_Wrist.setPose(.8763);
                s_Elevator.setPose(3.5472);
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
