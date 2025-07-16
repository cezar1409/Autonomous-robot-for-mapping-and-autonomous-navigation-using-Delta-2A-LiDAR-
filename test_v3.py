import LidarDelta2A
import signal
import time
import matplotlib.pyplot as plt
import math
import numpy as np

# Constants for navigation - Robot orientation corrected
MIN_DISTANCE = 100  # Very close obstacles only - 10cm in mm
SECTOR_ANGLES = {
    'front': (60, 120),     # 90° ± 30° (positive Y direction - front)
    'left': (330, 30),      # 0° ± 30° (positive X direction - left side of robot, right side of map)  
    'back': (240, 300),     # 270° ± 30° (negative Y direction - back)
    'right': (150, 210)     # 180° ± 30° (negative X direction - right side of robot, left side of map)
}

# Robot orientation and corner detection
robot_heading = 0  # This will be updated based on movement direction
blocked_directions = []  # Track which directions have been blocked recently
turn_cooldown = 0  # Prevent rapid turning
last_turn_time = 0

def calculate_obstacle_forces(points):
    """Calculate repulsive forces from obstacles for obstacle avoidance"""
    # Robot dimensions for safety margin
    robot_radius = 120  # mm - tighter safety radius around robot (12cm)
    
    force_x = 0
    force_y = 0
    
    MIN_RELIABLE_DISTANCE = 50   # mm - very close readings (5cm minimum)
    MAX_INFLUENCE_DISTANCE = 300  # mm - detect obstacles within 30cm
    
    for point in points:
        angle = point[0]
        distance = point[1]
        
        # Skip unreliable or very distant points
        if distance < MIN_RELIABLE_DISTANCE or distance > MAX_INFLUENCE_DISTANCE:
            continue
        
        # Convert to cartesian coordinates
        obs_x = distance * math.cos(math.radians(angle))
        obs_y = distance * math.sin(math.radians(angle))
        
        # Calculate repulsive force (stronger for closer obstacles)
        if distance > robot_radius:
            force_magnitude = (MAX_INFLUENCE_DISTANCE - distance) / distance
            # Force points away from obstacle
            force_x -= obs_x * force_magnitude / distance
            force_y -= obs_y * force_magnitude / distance
    
    return force_x, force_y

def detect_corner_situation(min_distances):
    """Detect if robot is in a corner or dead-end situation requiring 90-degree turn"""
    BLOCKED_THRESHOLD = 400  # mm - consider direction blocked if obstacle closer than 40cm
    
    front_blocked = min_distances['front'] < BLOCKED_THRESHOLD
    left_blocked = min_distances['left'] < BLOCKED_THRESHOLD  
    right_blocked = min_distances['right'] < BLOCKED_THRESHOLD
    
    # Corner situations that require 90-degree turn:
    # 1. Front + Left blocked (turn right)
    # 2. Front + Right blocked (turn left)  
    # 3. Front + Left + Right all blocked (turn around - 180 degrees)
    
    if front_blocked and left_blocked and right_blocked:
        return "turn_around"  # 180-degree turn
    elif front_blocked and left_blocked:
        return "turn_right"   # 90-degree right turn
    elif front_blocked and right_blocked:
        return "turn_left"    # 90-degree left turn
    
    return None  # No corner situation detected

def get_safe_direction(force_x, force_y):
    """Convert force vector to safe movement direction - NO BACKWARD MOVEMENT"""
    # Robot orientation: +Y=front, -Y=back, +X=left side, -X=right side
    if abs(force_x) < 0.1 and abs(force_y) < 0.1:
        return "forward"  # No significant obstacles, move forward (+Y)
    
    # Prioritize lateral movement to go around obstacles - NEVER go backward
    if abs(force_x) > abs(force_y) * 0.5:  # Prefer lateral movement
        if force_x > 0:
            return "left"   # Move to robot's left (+X) to avoid obstacles on right
        else:
            return "right"  # Move to robot's right (-X) to avoid obstacles on left
    else:
        # Only consider forward movement - never backward
        if force_y >= 0:
            return "forward"  # Move forward (+Y) 
        else:
            # If force suggests backward but we don't go backward, choose lateral
            if abs(force_x) > 0.1:
                return "left" if force_x > 0 else "right"
            else:
                return "forward"  # Default to forward  

def get_direction(points):
    """Analyze LIDAR data and decide navigation direction with corner detection"""
    global robot_heading, blocked_directions, turn_cooldown, last_turn_time
    
    # Reduce turn cooldown over time
    if turn_cooldown > 0:
        turn_cooldown -= 1
    
    # Analyze obstacles in each sector
    sectors = {
        'front': [],
        'right': [],
        'back': [],
        'left': []
    }
    
    MIN_RELIABLE_DISTANCE = 50   # mm - accept very close readings
    SAFE_DISTANCE = 300  # mm - minimum safe distance for movement (30cm)
    
    # Group points by sector
    for point in points:
        angle = point[0]
        distance = point[1]
        
        if distance < MIN_RELIABLE_DISTANCE:
            continue
            
        for sector, (start, end) in SECTOR_ANGLES.items():
            # Handle sector angle checking based on robot orientation
            if sector == 'left' and (angle >= 330 or angle <= 30):
                # Left sector wraps around 0° (330° to 30°)
                sectors[sector].append(distance)
            elif sector != 'left' and start <= angle <= end:
                # Other sectors are normal ranges
                sectors[sector].append(distance)
    
    # Calculate minimum distances for each sector
    min_distances = {}
    for sector, distances in sectors.items():
        if distances:
            min_distances[sector] = min(distances)
        else:
            min_distances[sector] = float('inf')
    
    # Check for corner situation first (higher priority than normal navigation)
    corner_situation = detect_corner_situation(min_distances)
    
    if corner_situation and turn_cooldown == 0:
        # Execute corner turn logic
        if corner_situation == "turn_around":
            print("🔄 Corner Detection: DEAD END - Turning around 180°")
            turn_cooldown = 10  # Prevent rapid turning
            return "turn_around"
        elif corner_situation == "turn_right":
            print("🔄 Corner Detection: CORNER - Turning right 90°")
            turn_cooldown = 8
            return "turn_right_90"
        elif corner_situation == "turn_left":
            print("🔄 Corner Detection: CORNER - Turning left 90°")
            turn_cooldown = 8
            return "turn_left_90"
    
    # If no corner situation, use normal obstacle avoidance
    force_x, force_y = calculate_obstacle_forces(points)
    preferred_direction = get_safe_direction(force_x, force_y)
    
    # Verify preferred direction is safe - NO BACKWARD MOVEMENT
    direction_map = {
        'forward': 'front',   # +Y direction
        'right': 'right',     # -X direction (robot's right, map's left)
        'left': 'left',       # +X direction (robot's left, map's right)
        # Removed 'backward': 'back' - NO BACKWARD MOVEMENT ALLOWED
    }
    
    preferred_sector = direction_map.get(preferred_direction, 'front')
    
    if min_distances[preferred_sector] > SAFE_DISTANCE:
        print(f"Obstacle Avoidance: {preferred_direction}")
        return preferred_direction
    
    # If preferred direction is blocked, find alternative safe direction (NO BACKWARD)
    safe_directions = []
    for direction, sector in direction_map.items():
        if min_distances[sector] > SAFE_DISTANCE:
            safe_directions.append(direction)
    
    if safe_directions:
        # Priority: 1) Forward, 2) Left/Right to go around obstacles
        if "forward" in safe_directions:
            print("Obstacle Avoidance: forward (fallback)")
            return "forward"
        elif "left" in safe_directions:
            print("Obstacle Avoidance: left (go around)")
            return "left"
        elif "right" in safe_directions:
            print("Obstacle Avoidance: right (go around)")
            return "right"
        else:
            chosen = safe_directions[0]
            print(f"Obstacle Avoidance: {chosen} (fallback)")
            return chosen
    else:
        print("Obstacle Avoidance: STOP - all close directions blocked")
        return "stop"

port = 'COM6'
lidar = LidarDelta2A.LidarDelta2A(port, baudrate = 230400) 

points = np.zeros((16*52,2))

def on_close(event):
    print('Closed window!')
    global ready
    ready = 0

def signal_handler(sig, frame):
    print('Pressed Ctrl+C!')
    global ready
    ready = 0

signal.signal(signal.SIGINT, signal_handler)

# Create a larger figure with better clarity
plt.style.use('dark_background')  # Better contrast
fig, ax = plt.subplots(figsize=(12, 12))  # Larger window

fig.canvas.mpl_connect('close_event', on_close)

# Enhanced scatter plot
ln = ax.scatter(points[:,0], points[:,1], c='lime', s=2, animated=True)  # Smaller points, better color
ax.set_xlim([-8000,8000])  # Larger view
ax.set_ylim([-8000,8000])
ax.grid(True, linestyle='--', alpha=0.5)  # Add grid
ax.set_title('LIDAR Navigation View', fontsize=14)
ax.set_xlabel('X Distance (mm)', fontsize=12)
ax.set_ylabel('Y Distance (mm)', fontsize=12)

# Add robot representation at center (NO TRIANGLE ON ROBOT) - ROTATED 90 DEGREES
robot_length = 300  # mm
robot_width = 150   # mm

from matplotlib.patches import Rectangle, Wedge
# Swap width and length to rotate robot 90 degrees
robot_body = Rectangle((-robot_width/2, -robot_length/2), robot_width, robot_length, 
                      linewidth=2, edgecolor='cyan', facecolor='blue', alpha=0.5)
ax.add_patch(robot_body)

# Front sector will be created when needed

def update_front_sector():
    """Create the yellow front sector - Robot front is on positive Y axis"""
    # Robot front is on positive Y axis (90° in matplotlib coordinates)
    front_angle = 90  # Fixed front direction (positive Y)
    sector_width = 60  # 60 degree sector (±30 from center)
    
    # Create wedge showing robot's front direction (60° to 120°)
    front_sector = Wedge((0, 0), 8000, front_angle - sector_width/2, 
                        front_angle + sector_width/2, alpha=0.15, color='yellow')
    ax.add_patch(front_sector)
    return front_sector

# Add robot body and front sector to static background
robot_body = Rectangle((-robot_width/2, -robot_length/2), robot_width, robot_length, 
                      linewidth=2, edgecolor='cyan', facecolor='blue', alpha=0.5)
ax.add_patch(robot_body)

# Add front sector
update_front_sector()

plt.show(block=False)
plt.pause(0.1)

bg = fig.canvas.copy_from_bbox(fig.bbox)
ax.draw_artist(ln)
fig.canvas.blit(fig.bbox)

# Timing control - separate visualization from robot control
visualization_interval = 0.05  # 20 FPS for visualization updates
navigation_interval = 0.5      # 2 Hz for robot navigation commands (much slower!)

t = time.time()
nxt_visualization = time.time() + visualization_interval
nxt_navigation = time.time() + navigation_interval

ready = 1
current_direction = "stop"  # Track current robot command

print("🤖 === CEZAR'S AUTONOMOUS MOBILE ROBOT (AMR) === 🤖")
print("Starting LiDAR navigation - Robot control at 2Hz, Visualization at 20Hz")

while ready:
    data = lidar.handleData()
    
    # Process points and identify approaching obstacles
    plot_points = []
    approaching_obstacles = []
    DANGER_DISTANCE = 300  # mm - mark obstacles closer than this as dangerous (30cm)
    
    for i,v in enumerate(data):
        x = v[1] * math.cos(math.radians(v[0]))
        y = v[1] * math.sin(math.radians(v[0]))
        distance = v[1]
        
        plot_points.append([x, y])
        points[i] = [x, y]
        
        # Mark obstacles that are approaching (within danger distance)
        if 50 < distance < DANGER_DISTANCE:  # Only close obstacles matter (5-30cm)
            approaching_obstacles.append([x, y])

    # Update visualization at higher frequency (20 FPS)
    if time.time() > nxt_visualization:
        nxt_visualization = time.time() + visualization_interval

        # Update visualization efficiently
        fig.canvas.restore_region(bg)
        
        # Update LiDAR points
        ln.set_offsets(plot_points)
        
        # Change colors for approaching obstacles
        colors = []
        for i, (x, y) in enumerate(plot_points):
            # Check if this point is an approaching obstacle
            is_danger = any(abs(x - ox) < 10 and abs(y - oy) < 10 for ox, oy in approaching_obstacles)
            if is_danger:
                colors.append('red')
            else:
                colors.append('lime')
        
        ln.set_color(colors)
        ax.draw_artist(ln)
        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()
    
    # Send navigation commands at lower frequency (2 Hz) to avoid overwhelming robot
    if time.time() > nxt_navigation:
        nxt_navigation = time.time() + navigation_interval
        
        # Calculate new direction
        new_direction = get_direction(data)
        
        # Only send command if direction changed (reduce unnecessary commands)
        if new_direction != current_direction:
            current_direction = new_direction
            print(f"Robot Command: {current_direction}")
            
            # TODO: Send actual command to robot here
            # send_robot_command(current_direction)  # Will send letter commands to Arduino
            
        # For mecanum wheels, robot body doesn't rotate - front direction stays fixed
        # Yellow sector shows the permanent front of the robot/LiDAR

plt.close("all")
lidar.stop()