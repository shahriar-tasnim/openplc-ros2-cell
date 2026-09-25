(* SelfTestEnable *)

PROGRAM openplc_ros2_pick_place
  VAR
    StartPB : BOOL AT %IX0.0 := FALSE;
    StopPB : BOOL AT %IX0.1 := FALSE;
    ResetPB : BOOL AT %IX0.2 := FALSE;
    AutoMode : BOOL AT %IX0.3 := FALSE;
    SafetyOK : BOOL AT %IX0.4 := FALSE;
    PartAtPick : BOOL AT %IX0.5 := FALSE;
    PlaceAreaClear : BOOL AT %IX0.6 := FALSE;
    CommunicationOK : BOOL AT %IX0.7 := FALSE;
    RobotHome : BOOL AT %IX1.0 := FALSE;
    RobotBusy : BOOL AT %IX1.1 := FALSE;
    RobotDone : BOOL AT %IX1.2 := FALSE;
    RobotFault : BOOL AT %IX1.3 := FALSE;
    ConveyorRun : BOOL AT %QX0.0 := FALSE;
    RobotStart : BOOL AT %QX0.1 := FALSE;
    RobotReset : BOOL AT %QX0.2 := FALSE;
    MoveHome : BOOL AT %QX0.3 := FALSE;
    CycleActive : BOOL AT %QX0.4 := FALSE;
    GreenLamp : BOOL AT %QX0.5 := FALSE;
    FaultLamp : BOOL AT %QX0.6 := FALSE;
    PLCHeartbeat : BOOL AT %QX0.7 := FALSE;
    RunLatch : BOOL := FALSE;
    CellState : INT := 0;
    CycleCount : DINT := 0;
    FaultCode : INT := 0;
    SelfTestPreviousStage : INT := 0;
    HeartbeatTimer : TON;
    RobotTimeout : TON;
    PartTimeout : TON;
    SelfTestEnable : BOOL := FALSE;
    SelfTestStart : BOOL := FALSE;
    SelfTestStartMemory : BOOL := FALSE;
    SelfTestStartPulse : BOOL := FALSE;
    SelfTestStageChanged : BOOL := FALSE;
    SelfTestRunning : BOOL := FALSE;
    SelfTestPass : BOOL := FALSE;
    SelfTestFail : BOOL := FALSE;
    SelfTestStage : INT := 0;
    SelfTestExpectedState : INT := 0;
    SelfTestErrorCode : INT := 0;
    SelfTestStepTimer : TON;
    SelfTestTimeout : TON;
    xStartPB : BOOL;
    xStopPB : BOOL;
    xResetPB : BOOL;
    xAutoMode : BOOL;
    xSafetyOK : BOOL;
    xPartAtPick : BOOL;
    xPlaceAreaClear : BOOL;
    xCommunicationOK : BOOL;
    xRobotHome : BOOL;
    xRobotBusy : BOOL;
    xRobotDone : BOOL;
    xRobotFault : BOOL;
    xRobotFault0 : BOOL;
  END_VAR

(* =========================================================
   OpenPLC + ROS 2 Simulated Pick-and-Place Cell
   Milestone 3: Debug-friendly automatic self-test
   ========================================================= *)


(* ---------------------------------------------------------
   Detect a rising edge on SelfTestStart
   --------------------------------------------------------- *)
SelfTestEnable := FALSE;

SelfTestStartPulse :=
    SelfTestStart
    AND NOT SelfTestStartMemory;

SelfTestStartMemory := SelfTestStart;


(* ---------------------------------------------------------
   Start a new automatic self-test
   --------------------------------------------------------- *)

IF SelfTestEnable AND SelfTestStartPulse THEN

    SelfTestRunning := TRUE;
    SelfTestPass := FALSE;
    SelfTestFail := FALSE;

    SelfTestStage := 1;
    SelfTestErrorCode := 0;

    RunLatch := FALSE;
    CellState := 0;
    CycleCount := 0;
    FaultCode := 0;

END_IF;


(* ---------------------------------------------------------
   Default: use the actual PLC inputs
   --------------------------------------------------------- *)

xStartPB := StartPB;
xStopPB := StopPB;
xResetPB := ResetPB;
xAutoMode := AutoMode;
xSafetyOK := SafetyOK;
xPartAtPick := PartAtPick;
xPlaceAreaClear := PlaceAreaClear;
xCommunicationOK := CommunicationOK;
xRobotHome := RobotHome;
xRobotBusy := RobotBusy;
xRobotDone := RobotDone;
xRobotFault := RobotFault;


(* ---------------------------------------------------------
   Self-test mode: replace the physical inputs with
   automatically generated test signals
   --------------------------------------------------------- *)

IF SelfTestEnable THEN

    (* Healthy default conditions *)

    xStartPB := FALSE;
    xStopPB := FALSE;
    xResetPB := FALSE;

    xAutoMode := TRUE;
    xSafetyOK := TRUE;
    xCommunicationOK := TRUE;

    xRobotHome := TRUE;
    xPlaceAreaClear := TRUE;

    xPartAtPick := FALSE;
    xRobotBusy := FALSE;
    xRobotDone := FALSE;
    xRobotFault := FALSE;


    IF SelfTestRunning THEN

        CASE SelfTestStage OF


            1:  (* INITIALIZE *)

                xResetPB := TRUE;


            2:  (* PRESS START *)

                xStartPB := TRUE;


            3:  (* PART ARRIVES *)

                xPartAtPick := TRUE;


            4:  (* ROBOT ACCEPTS COMMAND *)

                xPartAtPick := TRUE;
                xRobotBusy := TRUE;


            5:  (* ROBOT COMPLETES OPERATION *)

                xPartAtPick := TRUE;
                xRobotBusy := FALSE;
                xRobotDone := TRUE;


            6:  (* CLEAR ROBOT AND SENSOR SIGNALS *)

                xPartAtPick := FALSE;
                xRobotBusy := FALSE;
                xRobotDone := FALSE;


            ELSE

                xStartPB := FALSE;
                xPartAtPick := FALSE;
                xRobotBusy := FALSE;
                xRobotDone := FALSE;

        END_CASE;

    END_IF;

END_IF;


(* ---------------------------------------------------------
   PLC heartbeat
   --------------------------------------------------------- *)

HeartbeatTimer(
    IN := NOT HeartbeatTimer.Q,
    PT := T#500ms
);

IF HeartbeatTimer.Q THEN
    PLCHeartbeat := NOT PLCHeartbeat;
END_IF;


(* ---------------------------------------------------------
   Cell operation timers
   --------------------------------------------------------- *)

RobotTimeout(
    IN :=
        (CellState = 20)
        OR (CellState = 30)
        OR (CellState = 40),
    PT := T#10s
);

PartTimeout(
    IN :=
        (CellState = 10)
        AND RunLatch
        AND ConveyorRun
        AND NOT SelfTestEnable,
    PT := T#30s
);


(* ---------------------------------------------------------
   Safe output defaults
   --------------------------------------------------------- *)

ConveyorRun := FALSE;
RobotStart := FALSE;
RobotReset := FALSE;
MoveHome := FALSE;

CycleActive := FALSE;
GreenLamp := FALSE;
FaultLamp := FALSE;


(* ---------------------------------------------------------
   Highest-priority fault checks
   --------------------------------------------------------- *)

IF NOT xSafetyOK THEN

    RunLatch := FALSE;
    CellState := 900;
    FaultCode := 100;


ELSIF NOT xCommunicationOK THEN

    RunLatch := FALSE;
    CellState := 900;
    FaultCode := 101;


ELSIF xRobotFault THEN

    RunLatch := FALSE;
    CellState := 900;
    FaultCode := 102;

    RobotReset :=
        xResetPB
        AND xSafetyOK
        AND xCommunicationOK;


(* ---------------------------------------------------------
   Stop or leave automatic mode
   --------------------------------------------------------- *)

ELSIF xStopPB OR NOT xAutoMode THEN

    RunLatch := FALSE;

    IF CellState <> 900 THEN
        CellState := 0;
    END_IF;


(* ---------------------------------------------------------
   Main automatic state machine
   --------------------------------------------------------- *)

ELSE

    CASE CellState OF


        0:  (* IDLE *)

            RunLatch := FALSE;

            IF xSafetyOK
                AND xCommunicationOK
                AND NOT xRobotHome
                AND NOT xRobotBusy
                AND NOT xRobotFault
            THEN
                MoveHome := TRUE;
            END_IF;

            IF xStartPB
                AND xAutoMode
                AND xSafetyOK
                AND xCommunicationOK
                AND NOT xRobotFault
            THEN
                RunLatch := TRUE;
                FaultCode := 0;
                CellState := 10;
            END_IF;


        10: (* WAIT FOR PART *)

            RunLatch := TRUE;
            ConveyorRun := TRUE;

            IF xPartAtPick THEN

                ConveyorRun := FALSE;
                CellState := 20;

            ELSIF PartTimeout.Q THEN

                RunLatch := FALSE;
                FaultCode := 203;
                CellState := 900;

            END_IF;


        20: (* CHECK ROBOT PERMISSIVES *)

            RunLatch := TRUE;
            ConveyorRun := FALSE;

            IF xRobotHome
                AND xPlaceAreaClear
                AND NOT xRobotBusy
                AND NOT xRobotFault
            THEN

                CellState := 30;

            ELSIF RobotTimeout.Q THEN

                RunLatch := FALSE;
                FaultCode := 204;
                CellState := 900;

            END_IF;


        30: (* SEND ROBOT START COMMAND *)

            RunLatch := TRUE;
            ConveyorRun := FALSE;
            RobotStart := TRUE;

            IF xRobotBusy THEN

                RobotStart := FALSE;
                CellState := 40;

            ELSIF RobotTimeout.Q THEN

                RunLatch := FALSE;
                RobotStart := FALSE;
                FaultCode := 201;
                CellState := 900;

            END_IF;


        40: (* WAIT FOR ROBOT COMPLETION *)

            RunLatch := TRUE;
            ConveyorRun := FALSE;
            RobotStart := FALSE;

            IF xRobotDone AND NOT xRobotBusy THEN

                CycleCount := CycleCount + 1;
                CellState := 50;

            ELSIF RobotTimeout.Q THEN

                RunLatch := FALSE;
                FaultCode := 202;
                CellState := 900;

            END_IF;


        50: (* CYCLE COMPLETE *)

            RunLatch := TRUE;
            ConveyorRun := FALSE;
            RobotStart := FALSE;

            IF NOT xRobotDone
                AND NOT xPartAtPick
            THEN
                CellState := 10;
            END_IF;


        900: (* FAULT *)

            RunLatch := FALSE;
            ConveyorRun := FALSE;
            RobotStart := FALSE;

            RobotReset :=
                xResetPB
                AND xSafetyOK
                AND xCommunicationOK;

            IF xResetPB
                AND xSafetyOK
                AND xCommunicationOK
                AND NOT xRobotFault
            THEN
                FaultCode := 0;
                CellState := 0;
            END_IF;


        ELSE

            RunLatch := FALSE;
            FaultCode := 999;
            CellState := 900;

    END_CASE;

END_IF;


(* ---------------------------------------------------------
   Status outputs
   --------------------------------------------------------- *)

CycleActive :=
    RunLatch
    AND (CellState >= 10)
    AND (CellState < 900);

GreenLamp := CycleActive;
FaultLamp := (CellState = 900);


(* ---------------------------------------------------------
   Expected PLC state for each self-test stage
   --------------------------------------------------------- *)

CASE SelfTestStage OF

    1:
        SelfTestExpectedState := 0;

    2:
        SelfTestExpectedState := 10;

    3:
        SelfTestExpectedState := 30;

    4:
        SelfTestExpectedState := 40;

    5:
        SelfTestExpectedState := 50;

    6:
        SelfTestExpectedState := 10;

    7:
        SelfTestExpectedState := 0;

    ELSE
        SelfTestExpectedState := 0;

END_CASE;


(* ---------------------------------------------------------
   Hold every successful stage for one second.
   This makes it visible in the debugger.
   --------------------------------------------------------- *)

(* Detect when the test enters a new stage *)
SelfTestStageChanged :=
    SelfTestStage <> SelfTestPreviousStage;

SelfTestPreviousStage := SelfTestStage;


(* Reset the timer for one PLC scan after every stage change *)
SelfTestStepTimer(
    IN :=
        SelfTestRunning
        AND NOT SelfTestStageChanged
        AND (CellState = SelfTestExpectedState),
    PT := T#1s
);


(* ---------------------------------------------------------
   Overall self-test timeout
   --------------------------------------------------------- *)

SelfTestTimeout(
    IN := SelfTestRunning,
    PT := T#20s
);


(* ---------------------------------------------------------
   Advance the automatic test
   --------------------------------------------------------- *)

IF SelfTestRunning THEN

    IF CellState = 900 THEN

        SelfTestRunning := FALSE;
        SelfTestPass := FALSE;
        SelfTestFail := TRUE;

        SelfTestErrorCode := FaultCode;


    ELSIF SelfTestTimeout.Q THEN

        SelfTestRunning := FALSE;
        SelfTestPass := FALSE;
        SelfTestFail := TRUE;

        (* 901 means timeout at stage 1,
           902 means timeout at stage 2, etc. *)

        SelfTestErrorCode := 900 + SelfTestStage;


    ELSE

        CASE SelfTestStage OF


            1:

                IF SelfTestStepTimer.Q THEN
                    SelfTestStage := 2;
                END_IF;


            2:

                IF SelfTestStepTimer.Q THEN
                    SelfTestStage := 3;
                END_IF;


            3:

                IF SelfTestStepTimer.Q THEN
                    SelfTestStage := 4;
                END_IF;


            4:

                IF SelfTestStepTimer.Q THEN
                    SelfTestStage := 5;
                END_IF;


            5:

                IF SelfTestStepTimer.Q THEN

                    IF CycleCount = 1 THEN
                        SelfTestStage := 6;
                    ELSE
                        SelfTestRunning := FALSE;
                        SelfTestFail := TRUE;
                        SelfTestErrorCode := 805;
                    END_IF;

                END_IF;


            6:

                IF (CellState = 10)
                    AND (CycleCount = 1)
                    AND ConveyorRun
                    AND NOT RobotStart
                    AND NOT FaultLamp
                THEN

                    SelfTestStage := 7;
                    SelfTestRunning := FALSE;
                    SelfTestPass := TRUE;
                    SelfTestFail := FALSE;
                    SelfTestErrorCode := 0;

                    (* Finish the test in a safe idle condition *)
                    RunLatch := FALSE;
                    CellState := 0;


                END_IF;

        END_CASE;

    END_IF;

END_IF;

END_PROGRAM