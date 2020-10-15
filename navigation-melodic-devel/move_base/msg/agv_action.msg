Header header
uint8 action
uint8 ACTION_MANUAL         = 0   # The goal has yet to be processed by the action server
uint8 ACTION_MOVE_BASE      = 1   # The goal has yet to be processed by the action server
uint8 ACTION_INITIAL_POSE   = 2   # The goal is currently being processed by the action server
uint8 ACTION_CHARGING_IN    = 3   # The goal received a cancel request after it started executing
uint8 ACTION_CHARGING_OUT   = 4   # The goal received a cancel request after it started executing
uint8 ACTION_LIFT_UP        = 5   # The goal received a cancel request after it started executing
uint8 ACTION_LIFT_DOWN      = 6   # The goal received a cancel request after it started executing

uint8 status
uint8 STATUS_PENDING         = 0   # The goal has yet to be processed by the action server
uint8 STATUS_ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 STATUS_PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 STATUS_SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 STATUS_ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 STATUS_REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 STATUS_PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 STATUS_RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 STATUS_RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 STATUS_LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string action_id
