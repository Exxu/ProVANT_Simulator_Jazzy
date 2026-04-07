# This file is part of the ProVANT simulator project.
# Licensed under the terms of the MIT open source license. More details at
# https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md

# This message contains information common to all ProVANT Simulator messages.
# It is designed to allow the collection of performance metrics from all the
# nodes inside the control group.

# The step in which the message was originated
uint32 step 0
# The wall time in which this message was generated
builtin_interfaces/Time timestamp
# The time it took to compute the relevant function
builtin_interfaces/Duration computation_time
