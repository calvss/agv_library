import random

class Vehicle:
    def __init__(self, wheelRadius, mass, friction, dt, # SI Units
                 initialX = 0, initialY = 0, 
                 initialVx = 0, initialVy = 0, 
                 initialOmegaX = 0, initialOmegaY = 0,  # wheel rotation speed in rad/s
                 errorMeanX = 0.01, errorMeanY = 0.01, 
                 errorSDX = 0.005, errorSDY = 0.005
                 ieeeErrorSD = 0.1):
        self.x = initialX
        self.y = initialY
        self.vx = initialVx
        self.vy = initialVy
        self.omegaX = initialOmegaX
        self.omegaY = initialOmegaY
        self.wheelRadius = wheelRadius
        self.mass = mass
        self.friction = friction
        self.dt = dt
        
        self.encoderX = 0
        self.encoderY = 0
        self.ieee = [0, 0]
        
    def setMotor(self, omegaX, omegaY):
        self.omegaX = omegaX
        self.omegaY = omegaY
        
    def updateModel(self):
        newVx = self.wheelRadius * self.omegaX
        newVy = self.wheelRadius * self.omegaY
        
        # there is only 1 traction force since x and y are assumed to have 
        # the same friction coeff.
        tractionForce = self.friction * self.mass * 9.80665 #should be enough decimal places for most applications
        acceleration = tractionForce / self.mass
        
        # dv is the maximum change in velocity per unit time permissible by tyre friction and mass
        # this is to simulate tyre slippage since velocity cannot instantly change
        dv = acceleration * self.dt
        
        # if the absolute difference between desired and actual velocity 
        # is greater than dv, increment velocity by dv
        if (abs(newVx - self.vx) > dv): 
            if newVx > self.vx:
                self.vx = self.vx + dv
            else:
                self.vx = self.vx - dv
        # else (if the difference in desired vs actual is small) 
        # just set the velocity directly
        else:                           
            self.vx = newVx
            
        if (abs(newVy - self.vy) > dv):
            if newVy > self.vy:
                self.vy = self.vy + dv
            else:
                self.vy = self.vy - dv
        else:
            self.vy = newVy
        
        # add random error to simulate uneven ground and slipping wheel
        self.x += (self.vx  + random.gauss(errorMeanX, errorSDX)) * self.dt
        self.y += (self.vy  + random.gauss(errorMeanY, errorSDY)) * self.dt
        
        self.encoderX += self.omegaX * self.dt
        self.encoderY += self.omegaY * self.dt
        
        # add random error to simulate imprecise GPS
        self.ieee[0] = self.x + random.gauss(0, ieeeErrorSD)
        self.ieee[1] = self.y + random.gauss(0, ieeeErrorSD)
        
    # encoder counts every 0.5 degrees, so convert from rad to deg, 
    # then multiply by 2    
    def readEncoder(self):              
        countsX = 2 * self.encoderX * 180 / 3.14159265359
        countsY = 2 * self.encoderY * 180 / 3.14159265359
        
        return (countsX, countsY)
        
    # IEEE simulates a GPS reading
    def readIEEE(self):
        return (self.ieee[0], self.ieee[1])
    
    # read the true position
    def readActual(self):
        return (self.x, self.y)