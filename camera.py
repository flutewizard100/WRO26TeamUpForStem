from ntcore import NetworkTableInstance
import time

class Camera:
    def __init__(self):

        # NT4 setup
        self.ntinst = NetworkTableInstance.getDefault()
        self.ntinst.startClient4("limelight-client")
        self.ntinst.setServer("172.29.0.1")

        self.table = self.ntinst.getTable("limelight")

        self.pipeline = 0
        self.horizontal = 0
        self.vertical = 0
        self.ta = 0
        self.tl = 0

    def setPipeline(self, p):
        if self.pipeline != p:
            self.pipeline = p

            # NT4 entry write
            self.table.getEntry("pipeline").setDouble(p)

    def update(self):

        self.horizontal = self.table.getEntry("tx").getDouble(0)
        self.vertical = self.table.getEntry("ty").getDouble(0)
        self.ta = self.table.getEntry("ta").getDouble(0)
        self.tl = self.table.getEntry("tl").getDouble(0)

    def getData(self):
        return {
            "horizontal angle": self.horizontal,
            "vertical angle": self.vertical,
            "ta": self.ta,
            "tl": self.tl
        }