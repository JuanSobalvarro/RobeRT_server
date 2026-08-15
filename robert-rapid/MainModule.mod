MODULE MainModule
    CONST jointtarget ZERO:=[[0,0,0,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST robtarget origin:=[[450.00,0.00,500.00],[4.14816E-8,6.1133E-9,-1,-2.53589E-16],[0,0,-1,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    CONST string IP_ADDR := "192.168.125.1";
    CONST num PORT := 5000;

    VAR speeddata move_speed := v100;
    VAR zonedata move_zone := fine;

    ! workspace limits defined by corner positions (vectors essentially), this should be manually adjusted in rapid based on the robot's workspace
    ! for security reasons
    VAR pos corner_min := [-500, -500, 600];
    VAR pos corner_max := [500, 500, 400];

    VAR shapedata workspace_volume;
    VAR wzstationary wz_workspace_limit;

    PROC InitializeWorkspaceLimit()
        WZBoxDef \Outside, workspace_volume, corner_min, corner_max;
        WZLimSup \Stat, wz_workspace_limit, workspace_volume;
        TPWrite "Workspace limit initialized.";
    ENDPROC

    PROC main()
        TPWrite "Initiating at origin...";
        MoveAbsJ ZERO, v100, fine, tool0;

        InitializeWorkspaceLimit;

        SocketServerTask;
    ENDPROC
ENDMODULE
