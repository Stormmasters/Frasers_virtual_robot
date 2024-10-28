package org.firstinspires.ftc.teamcode;

public class movementType{
    private String movementType;
    public movementType(String movementType){
        if (isValidmovementType(movementType)){
            this.movementType = movementType;
        }
        else{
            throw new IllegalArgumentException("Invalid movementType: " + movementType);
        }
    }
    private boolean isValidmovementType(String movementType){
        return movementType.equalsIgnoreCase("Strafe_or_forward_then_45deg") || movementType.equalsIgnoreCase("straightLine");
    }
    public String getmovementtype(){
        return movementType;
    }
}
