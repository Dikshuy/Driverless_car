class SumoSupervisorPlugin:
    def __init__(self, supervisor, traci, net):
        self.traci = traci
        self.runned = 0
    def run(self, duration):
        self.runned = self.runned + duration
        print self.traci.vehicle.getIDList()
        for i in self.traci.vehicle.getIDList():
            x = self.traci.vehicle.getPosition(i)[0]
            y = self.traci.vehicle.getPosition(i)[1]
            print(i,x,y) 
           
