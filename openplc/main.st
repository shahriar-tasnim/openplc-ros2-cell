PROGRAM main
  VAR_OUTPUT
    test_output : BOOL;
  END_VAR
  VAR
    blink_timer : TON;
  END_VAR

blink_timer(
    IN := NOT(blink_timer.Q),
    PT := T#20ms
);

IF blink_timer.Q THEN
    test_output := NOT(test_output);
END_IF;

END_PROGRAM