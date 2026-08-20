import sys
import time

# Point directly to your compiled ROS 2 library folder
sys.path.append('/home/pravin_workstation/aura_robot/install/aura_control/lib/aura_control')

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    print("Still can't find Rosmaster_Lib. Make sure the path is correct!")
    sys.exit(1)

print("Connecting to Board...")
# Make sure this port matches what you used to fix the first error!
bot = Rosmaster(com='/dev/ttyUSB0') 

print("Spinning all motors at speed 150...")
bot.set_motor(150, 150, 150, 150)
time.sleep(3)

print("Stopping motors...")
bot.set_motor(0, 0, 0, 0)
print("Test Complete.")