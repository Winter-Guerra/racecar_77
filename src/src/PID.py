import csv
class PIDController:
    def __init__(self,time, Kp=0, Ki=0, Kd=0):
        self._Kp = Kp
        self._Kd = Kd
        self._Ki = Ki
        #self._logHandle = open("log.csv", "a")
    
        self._integrator = 0
        self._error=0
        self._lastTime = time
        self._startTime = time
    def update(self, error, time):
        P = self._Kp * error
        dt = time - self._lastTime
        dt = dt.to_sec()
        self._integrator += error * dt
        I = self._integrator * self._Ki
        errDiff = error - self._error
        diff = errDiff/dt if dt != 0 else 0
        D = self._Kd * diff
        PID = P + I + D
        timeSince = time - self._startTime
        #self.logError(error, timeSince)
        self._lastTime = time
        self._error = error        
        return PID
        #print str(self._Kp)
        #print str(D)
    def logError(self, error, time):
        self._logHandle.write(str(time) + "," + str(error) + "\n")
        
