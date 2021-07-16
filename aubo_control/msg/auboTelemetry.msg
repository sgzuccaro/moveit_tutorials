#Header header 
float32[6] angle # degrees
float32[6] vel # deg/s
float32[6] current # amps
#time armReadTimestamp 
time startSyncTime 
uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
uint8 bufferHealth