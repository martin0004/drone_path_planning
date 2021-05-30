# SIMULATOR

# Udacity flight simulator connection port (string)
PORT = "5760"
# Udacity flight simulator IP (string)
IP = "127.0.0.1"

# MAP & OBSTACLES

# Size of cell discritization in the 2D map (float, meters).
CELL_SIZE = 1.0
# CSV file containing obstacle centers and half-sizes.
OBSTACLE_DATA_FILE = "./obstacles/colliders.csv"

# PLANNING

# Altitude the drone will fly at during its mission (positive up, float, meters)
DRONE_MAX_ALTITUDE = 40.0
# Distance the drone must keep from walls (float, meters)
DRONE_SAFETY_DISTANCE = 5.0
# Number of states to select in world to build state_space (int)
N_STATES = 500
# Max search distance around a node for the RRT algorithm [float, meters]
MAX_SEARCH_DISTANCE = 100.0
# Distance within which the drone is assumed to have reached a waypoint (float, meters)
DEADBAND = 5.0

# LOGS

# Logs directory
LOGS_DIRECTORY="logs"
# Telemetry log file
TLOG_NAME = "t.log"

