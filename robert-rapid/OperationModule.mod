MODULE OperationModule
    VAR robtarget tempRobTarget;
    VAR jointtarget tempJointTarget;

    FUNC bool IsSafeTarget(\robtarget rtarget, \jointtarget jtarget)
        VAR jointtarget test_joints;
        VAR robtarget test_robtarget;

        IF (Present(rtarget)) THEN
            ! If checking a Cartesian target, convert to joints
            test_joints := CalcJointT(rtarget, tool0 \WObj:=wobj0);
        ENDIF

        IF (Present(jtarget)) THEN
            ! If checking a Joint target, convert to Cartesian
            test_robtarget := CalcRobT(jtarget, tool0 \WObj:=wobj0);
        ENDIF

        RETURN TRUE;
    ERROR
        ! Catch unreachable coordinates or singularities safely without moving
        IF ERRNO = ERR_ROBLIMIT THEN
            TPWrite "PRE-CHECK WARNING: Target unreachable or singular!";
            RETURN FALSE;
        ENDIF
        RETURN FALSE;
    ENDFUNC

    PROC ExecuteAction(string action_cmd, \robtarget target_coords, \robtarget circular_extra_target, \jointtarget joint_target, \num speed, \zonedata zone)
        TPWrite "Action executed at time: " + CTime();

        TEST action_cmd
            CASE "PING":
                SendResponse "NACK|This ping should be answered at the middleware level, not here!";
            CASE "PINGR":
                SendResponse "ACK|PONGRUWU";

            CASE "MoveL":
                IF Present(target_coords) THEN
                    IF IsSafeTarget(\rtarget:=target_coords) THEN
                        MoveL target_coords, move_speed, move_zone, tool0;
                        SendResponse "ACK|MoveL";
                    ELSE
                        TPWrite "Error: MoveL target unreachable or singular.";
                        SendResponse "NACK|SINGULARITY_OR_LIMIT";
                    ENDIF
                ELSE
                    TPWrite "Error: Missing coordinates for MoveL.";
                    SendResponse "NACK|MISSING_COORDS";
                ENDIF
                RETURN;

            CASE "MoveJ":
                IF Present(target_coords) THEN
                    IF IsSafeTarget(\rtarget:=target_coords) THEN
                        MoveJ target_coords, move_speed, move_zone, tool0;
                        SendResponse "ACK|MoveJ";
                    ELSE
                        TPWrite "Error: MoveJ target unreachable or singular.";
                        SendResponse "NACK|SINGULARITY_OR_LIMIT";
                    ENDIF
                ELSE
                    TPWrite "Error: Missing coordinates for MoveJ.";
                    SendResponse "NACK|MISSING_COORDS";
                ENDIF
                RETURN;

            CASE "MoveAbsJ":
                IF Present(joint_target) THEN
                    IF IsSafeTarget(\jtarget:=joint_target) THEN
                        MoveAbsJ joint_target, move_speed, move_zone, tool0;
                        SendResponse "ACK|MoveAbsJ";
                    ELSE
                        TPWrite "Error: MoveAbsJ joints out of bounds.";
                        SendResponse "NACK|SINGULARITY_OR_LIMIT";
                    ENDIF
                ELSE
                    TPWrite "Error: Missing coordinates for MoveAbsJ.";
                    SendResponse "NACK|MISSING_COORDS";
                ENDIF
                RETURN;

            CASE "MoveC":
                IF Present(target_coords) AND Present(circular_extra_target) THEN
                    IF IsSafeTarget(\rtarget:=target_coords) AND IsSafeTarget(\rtarget:=circular_extra_target) THEN
                        MoveC target_coords, circular_extra_target, move_speed, move_zone, tool0;
                        SendResponse "ACK|MoveC";
                    ELSE
                        TPWrite "Error: MoveC targets unreachable or singular.";
                        SendResponse "NACK|SINGULARITY_OR_LIMIT";
                    ENDIF
                ELSE
                    TPWrite "Error: Missing coordinates for MoveC.";
                    SendResponse "NACK|MISSING_COORDS";
                ENDIF
                RETURN;

            CASE "SetSpeed":
                IF Present(speed) THEN
                    IF speed < 5 OR speed > MaxRobSpeed() THEN
                        TPWrite "Error: Speed value out of range (5 - " + ValToStr(MaxRobSpeed()) + "). Received: " + ValToStr(speed);
                        SendResponse "NACK|SPEED_OUT_OF_RANGE";
                        RETURN;
                    ENDIF
                    TPWrite "Action: Setting speed to : " + NumToStr(speed, 0) + " mm/s";
                    move_speed.v_tcp := speed;
                    SendResponse "ACK|SetSpeed";
                ELSE
                    TPWrite "Error: Missing speed value for SetSpeed.";
                    SendResponse "NACK|MISSING_SPEED";
                ENDIF
                RETURN;

            CASE "SetZone":
                TPWrite "Action: Setting zone...";
                move_zone := zone;
                SendResponse "ACK|SetZone";
                RETURN;

            CASE "ZERO":
                TPWrite "Action: Moving to zero position...";
                MoveAbsJ ZERO, move_speed, move_zone, tool0;
                SendResponse "ACK|ZERO";
                RETURN;

            CASE "GETSTATUS":
                TPWrite "Action: Getting robot status...";
                HandleGetState;
                RETURN;

            CASE "MoveLOffs":
                IF Present(target_coords) THEN
                    tempRobTarget := Offs(CRobT(\Tool:=tool0 \WObj:=wobj0), target_coords.trans.x, target_coords.trans.y, target_coords.trans.z);
                    IF IsSafeTarget(\rtarget:=tempRobTarget) THEN
                        MoveL tempRobTarget, move_speed, move_zone, tool0;
                        SendResponse "ACK|MOVE_L_OFFS";
                    ELSE
                        TPWrite "Error: Offset target hits singularity or boundary.";
                        SendResponse "NACK|SINGULARITY_OR_LIMIT";
                    ENDIF
                ELSE
                    TPWrite "Error: Missing offset deltas for MOVE_L_OFFS.";
                    SendResponse "NACK|MISSING_COORDS";
                ENDIF
                RETURN;

            CASE "MoveJOffs":
                IF Present(target_coords) THEN
                    tempRobTarget := Offs(CRobT(\Tool:=tool0 \WObj:=wobj0), target_coords.trans.x, target_coords.trans.y, target_coords.trans.z);
                    IF IsSafeTarget(\rtarget:=tempRobTarget) THEN
                        MoveJ tempRobTarget, move_speed, move_zone, tool0;
                        SendResponse "ACK|MOVE_J_OFFS";
                    ELSE
                        TPWrite "Error: Offset target hits singularity or boundary.";
                        SendResponse "NACK|SINGULARITY_OR_LIMIT";
                    ENDIF
                ELSE
                    TPWrite "Error: Missing offset deltas for MOVE_J_OFFS.";
                    SendResponse "NACK|MISSING_COORDS";
                ENDIF
                RETURN;

            DEFAULT:
                TPWrite "Error: Operation not defined for command: " + action_cmd;
                SendResponse "NACK|UNKNOWN_COMMAND";
                RETURN;
        ENDTEST

    ERROR
        IF ERRNO = ERR_SING_AREA THEN
            TPWrite "[WARN] Singularity area reached. Skipping movement...";
            SendResponse "NACK|SINGULARITY_SKIPPED";
            
            ! 1. Erase the problematic trajectory from the planner
            ClearPath;
            
            ! 2. Skip the failed instruction and continue the program
            TRYNEXT;
        ! Emergency fallback if an unexpected physical runtime exception still occurs
        ELSEIF ERRNO = ERR_ROBLIMIT THEN
            TPWrite "Out of bounds movement, returning to ZERO";
            SendResponse "NACK|ERROR ON EXECUTION. Out of bounds movement, returning to ZERO";
        ELSE
            TPWrite "IDK what occurred, returning to ZERO";
            SendResponse "NACK|ERROR ON EXECUTION.";
        ENDIF

        ClearPath;
        StorePath;
        ! ALWAYS use 'fine' for emergency return moves to prevent socket/zone freezes
        MoveAbsJ ZERO, v100, fine, tool0;
        RestoPath;

        RETURN;
    ENDPROC
ENDMODULE
