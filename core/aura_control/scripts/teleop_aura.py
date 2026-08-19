import sys, select, termios, tty
import time
from Rosmaster_Lib import Rosmaster

# --- ON-SCREEN INSTRUCTIONS ---
msg = """
🎮 Robot Keyboard Teleop 🎮
---------------------------
Moving around:
        W
    A   S   D
        X

W / X   : Increase / Decrease speed (Forward/Reverse)
A / D   : Steer Left / Steer Right
S       : Stop motors (Keep steering angle)
SPACE   : EMERGENCY STOP (Stop motors & Center steering)

CTRL-C to quit
---------------------------
"""

def getKey(settings):
    """Reads a single keypress from the terminal without blocking."""
    tty.setraw(sys.stdin.fileno())
    # 0.1 second timeout so the loop doesn't freeze waiting for a key
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    # Save standard terminal settings so we can restore them later
    settings = termios.tcgetattr(sys.stdin)
    
    # 1. Initialize the Yahboom ROS Control Board
    print("[*] Connecting to ROS Robot Control Board...")
    try:
        bot = Rosmaster(com='/dev/ttyUSB0') 
    except Exception as e:
        print(f"❌ Error connecting: {e}")
        return
        
    print(msg)

    # --- HARDWARE CONFIGURATION ---
    DRIVE_MOTOR_PORT = 1      # 1=M1, 2=M2, etc.
    STEER_SERVO_PORT = 4      # 4=S4
    
    STEER_CENTER = 90         
    STEER_MAX_LEFT = 45       
    STEER_MAX_RIGHT = 140     
    
    MAX_SPEED = 120
    # Steps control how much the speed/angle changes per keystroke
    SPEED_STEP = 20           
    ANGLE_STEP = 15           
    # ------------------------------

    # Starting states
    throttle = 0
    angle = STEER_CENTER

    try:
        while True:
            # Read keyboard input
            key = getKey(settings)
            
            # --- 1. PROCESS KEYSTROKES ---
            if key == 'w':
                throttle = min(throttle + SPEED_STEP, MAX_SPEED)
            elif key == 'x':
                throttle = max(throttle - SPEED_STEP, -MAX_SPEED)
            elif key == 's':
                throttle = 0
            elif key == 'a':
                angle = max(angle - ANGLE_STEP, STEER_MAX_LEFT) 
            elif key == 'd':
                angle = min(angle + ANGLE_STEP, STEER_MAX_RIGHT)
            elif key == ' ': # Spacebar
                throttle = 0
                angle = STEER_CENTER
            elif key == '\x03': # CTRL+C
                break
            
            # --- 2. SEND COMMANDS TO HARDWARE ---
            if DRIVE_MOTOR_PORT == 1:
                bot.set_motor(throttle, 0, 0, 0)
            elif DRIVE_MOTOR_PORT == 2:
                bot.set_motor(0, throttle, 0, 0)
            
            bot.set_pwm_servo(STEER_SERVO_PORT, angle)
            
            # --- 3. PRINT STATUS ---
            # Overwrite the same line in the terminal to show live stats
            if key != '':
                print(f"\r🚀 Speed: {throttle:4d}  |  🏎️ Steering Angle: {angle:3d}    ", end='')
                
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
        
    finally:
        # --- SAFETY CUTOFF ---
        print("\n\n🛑 Stopping robot...")
        bot.set_motor(0, 0, 0, 0)
        bot.set_pwm_servo(STEER_SERVO_PORT, STEER_CENTER)
        
        # Restore normal terminal behavior
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()import sys, select, termios, tty
import time
from Rosmaster_Lib import Rosmaster

# --- ON-SCREEN INSTRUCTIONS ---
msg = """
🎮 Robot Keyboard Teleop 🎮
---------------------------
Moving around:
        W
    A   S   D
        X

W / X   : Increase / Decrease speed (Forward/Reverse)
A / D   : Steer Left / Steer Right
S       : Stop motors (Keep steering angle)
SPACE   : EMERGENCY STOP (Stop motors & Center steering)

CTRL-C to quit
---------------------------
"""

def getKey(settings):
    """Reads a single keypress from the terminal without blocking."""
    tty.setraw(sys.stdin.fileno())
    # 0.1 second timeout so the loop doesn't freeze waiting for a key
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    # Save standard terminal settings so we can restore them later
    settings = termios.tcgetattr(sys.stdin)
    
    # 1. Initialize the Yahboom ROS Control Board
    print("[*] Connecting to ROS Robot Control Board...")
    try:
        bot = Rosmaster(com='/dev/ttyUSB0') 
    except Exception as e:
        print(f"❌ Error connecting: {e}")
        return
        
    print(msg)

    # --- HARDWARE CONFIGURATION ---
    DRIVE_MOTOR_PORT = 1      # 1=M1, 2=M2, etc.
    STEER_SERVO_PORT = 4      # 4=S4
    
    STEER_CENTER = 90         
    STEER_MAX_LEFT = 45       
    STEER_MAX_RIGHT = 140     
    
    MAX_SPEED = 120
    # Steps control how much the speed/angle changes per keystroke
    SPEED_STEP = 20           
    ANGLE_STEP = 15           
    # ------------------------------

    # Starting states
    throttle = 0
    angle = STEER_CENTER

    try:
        while True:
            # Read keyboard input
            key = getKey(settings)
            
            # --- 1. PROCESS KEYSTROKES ---
            if key == 'w':
                throttle = min(throttle + SPEED_STEP, MAX_SPEED)
            elif key == 'x':
                throttle = max(throttle - SPEED_STEP, -MAX_SPEED)
            elif key == 's':
                throttle = 0
            elif key == 'a':
                angle = max(angle - ANGLE_STEP, STEER_MAX_LEFT) 
            elif key == 'd':
                angle = min(angle + ANGLE_STEP, STEER_MAX_RIGHT)
            elif key == ' ': # Spacebar
                throttle = 0
                angle = STEER_CENTER
            elif key == '\x03': # CTRL+C
                break
            
            # --- 2. SEND COMMANDS TO HARDWARE ---
            if DRIVE_MOTOR_PORT == 1:
                bot.set_motor(throttle, 0, 0, 0)
            elif DRIVE_MOTOR_PORT == 2:
                bot.set_motor(0, throttle, 0, 0)
            
            bot.set_pwm_servo(STEER_SERVO_PORT, angle)
            
            # --- 3. PRINT STATUS ---
            # Overwrite the same line in the terminal to show live stats
            if key != '':
                print(f"\r🚀 Speed: {throttle:4d}  |  🏎️ Steering Angle: {angle:3d}    ", end='')
                
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
        
    finally:
        # --- SAFETY CUTOFF ---
        print("\n\n🛑 Stopping robot...")
        bot.set_motor(0, 0, 0, 0)
        bot.set_pwm_servo(STEER_SERVO_PORT, STEER_CENTER)
        
        # Restore normal terminal behavior
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
