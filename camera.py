import limelight
import limelightresults
import time

class LimelightTracker:
    def __init__(self):
        # Camera constants to natively calculate degrees from pixels
        self.CENTER_X = 160.0
        self.CENTER_Y = 120.0
        self.DEG_PER_PIXEL_X = 63.3 / 320.0
        self.DEG_PER_PIXEL_Y = 49.7 / 240.0
        
        # Initialize target storage
        self.red_target = None   # Will store {"tx": val, "ty": val}
        self.green_target = None # Will store {"tx": val, "ty": val}
        self.ll = None

        # Automatically discover the Limelight on the robot network
        discovered = limelight.discover_limelights()
        
        if not discovered:
            print("No Limelight found on the network!")
            return

        # Connect to the first discovered camera (Your original untouched line)
        self.ll = limelight.Limelight(discovered[0])
        print(f"Connected to Limelight at: {discovered[0]}")

    def update(self):
        """Grabs new data from the Limelight and calculates target coordinates."""
        if self.ll is None:
            return

        # Reset targets at the start of each frame update
        self.red_target = None
        self.green_target = None

        raw_results = self.ll.get_results()
        parsed_data = limelightresults.parse_results(raw_results)
        
        if parsed_data is not None:
            data = parsed_data.pythonOutputs  
            
            if data and len(data) >= 8:
                rx, ry, rw, rh = data[0], data[1], data[2], data[3]
                gx, gy, gw, gh = data[4], data[5], data[6], data[7]

                # 1. Process RED if target exists
                if rw > 0:
                    red_center_x = rx + (rw / 2.0)
                    red_center_y = ry + (rh / 2.0)
                    red_tx = (red_center_x - self.CENTER_X) * self.DEG_PER_PIXEL_X
                    red_ty = (self.CENTER_Y - red_center_y) * self.DEG_PER_PIXEL_Y
                    self.red_target = {"tx": red_tx, "ty": red_ty}
                
                # 2. Process GREEN if target exists
                if gw > 0:
                    green_center_x = gx + (gw / 2.0)
                    green_center_y = gy + (gh / 2.0)
                    green_tx = (green_center_x - self.CENTER_X) * self.DEG_PER_PIXEL_X
                    green_ty = (self.CENTER_Y - green_center_y) * self.DEG_PER_PIXEL_Y
                    self.green_target = {"tx": green_tx, "ty": green_ty}

    def get_red_target(self):
        """Returns red target dictionary {'tx': val, 'ty': val} if it exists, otherwise None."""
        return self.red_target

    def get_green_target(self):
        """Returns green target dictionary {'tx': val, 'ty': val} if it exists, otherwise None."""
        return self.green_target


# Example of how you can loop and run this class in your project:
# if __name__ == "__main__":
#     tracker = LimelightTracker()
    
#     while True:
#         tracker.update() # Refreshes internal targets
        
#         # Pull your values out of the class using the getter methods
#         red = tracker.get_red_target()
#         green = tracker.get_green_target()
        
#         if red:
#             print(f"Class Red -> tx: {red['tx']:.2f}°, ty: {red['ty']:.2f}°")
            
#         if green:
#             print(f"Class Green -> tx: {green['tx']:.2f}°, ty: {green['ty']:.2f}°")
            
#         if not red and not green:
#             print("Scanning... No targets found.")

#         time.sleep(0.02)
