from numpy.lib.function_base import median
from config.config import *
from enum import Enum
import msgpack
import time
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from utils.frames import *
from utils.mapping import *
from utils.planning import *


# ----- CLASSES  ----- #

class FlightPhases(Enum):

    MANUAL = 0
    ARMING = 1
    PLANNING = 2
    TAKEOFF = 3
    WAYPOINT = 4
    LANDING = 5
    DISARMING = 6


class AutonomousDrone(Drone):

    # ----- INIT ----- #

    def __init__(self, conn):

        # Initialize drone parameters

        super().__init__(conn, tlog_directory=LOGS_DIRECTORY, tlog_name=TLOG_NAME)
        
        # in_mission
        #     if True, state callbacks will be active.
        #     must be set to True when starting the autonomous mission
        #     and False just before switching back to manual after mission
        self.in_mission = False

        # plan_ready
        #    indicates a new plan was found and sent to the simulator
        self.plan_ready = False

        # waypoints
        #    array of waypoints (position commands) the drone must follow
        self.waypoints = np.array([])

        # target_position
        #    position drone is trying to reach
        #    local ECEF frame
        self.target_position = np.array([0.0, 0.0, 0.0])

        # Register callbacks

        self.register_callback(MsgID.STATE, self.callback_state)
        self.register_callback(MsgID.LOCAL_POSITION, self.callback_local_position)

        # Begin autonomous mission

        # Change in_mission and set flight phase to manual.
        # This triggers the state callback and launches the mission.

        print()
        print("----- MANUAL PHASE-----")
        self.in_mission = True
        self.flight_phase = FlightPhases.MANUAL


    # ----- CALLBACKS ----- #  

    def callback_state(self):

        if self.in_mission:

            if self.flight_phase == FlightPhases.MANUAL:

                self.take_control()
                self.arming_transition()

            elif self.flight_phase == FlightPhases.ARMING:

                if self.armed:
                    self.plan()

            elif self.flight_phase == FlightPhases.PLANNING:

                if self.plan_ready:
                    self.takeoff_transition()   

            elif self.flight_phase == FlightPhases.DISARMING:

                if not self.armed and not self.guided:
                    self.manual_transition()     


    def callback_local_position(self):

        if self.flight_phase == FlightPhases.TAKEOFF:

            if self.reached_target():
                self.waypoint_transition()

        elif self.flight_phase == FlightPhases.WAYPOINT:

            if self.reached_target():
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    self.landing_transition()   

        elif self.flight_phase == FlightPhases.LANDING:

            if self.reached_target():
                self.disarming_transition()              


    # ----- TRANSITIONS ----- #

    def arming_transition(self):

        print()
        print("----- ARMING PHASE ---")

        self.flight_phase = FlightPhases.ARMING
        self.arm()


    def plan(self):

        print()
        print("----- PLANNING PHASE -----")

        self.plan_ready = False
        self.flight_phase = FlightPhases.PLANNING

        # DEFINE THE PLANNING PROBLEM

        # 2.5D GRID DISCRETIZATION

        print()
        print("2.5D grid discretization")

        print("Loading obstacle data...")
        obstacle_data = get_obstacle_data(OBSTACLE_DATA_FILE)

        print("Loading 2D map...")
        # 2D map at ground level
        grid_ground = get_grid(obstacle_data, 0.0, DRONE_SAFETY_DISTANCE, CELL_SIZE)
        # 2D map at drone max altitude
        grid_max = get_grid(obstacle_data, DRONE_MAX_ALTITUDE, DRONE_SAFETY_DISTANCE, CELL_SIZE)
        
        # VARIABLES FOR FRAME TRANSFORMS

        north_min, _, east_min, _ = get_local_map_size(obstacle_data)

        # HOME

        print()
        print("Home")
        print()

        global_home = get_global_home(OBSTACLE_DATA_FILE)
        local_home = global_to_local(global_home, global_home)
        grid_home = global_to_grid(global_home, global_home, north_min, east_min, CELL_SIZE)

        # Setting home_position attribute
        # Note: the set_home_position function takes its input as (lon, lat, alt)
        #       which is different from the global frame I used (lat, lon, alt)
        self.set_home_position(global_home[1], global_home[0], global_home[2])
        
        print("global frame ", global_home)
        print("local frame  ", local_home)
        print("grid frame   ", grid_home)

        # START STATE

        print()
        print("Start state")
        print()

        global_start = tuple(self.global_position)
        local_start = tuple(self.local_position)
        grid_start = local_to_grid(local_start, north_min, east_min, CELL_SIZE)

        print("global frame ", global_start)
        print("local_frame  ", local_start)
        print("grid_frame   ", grid_start)

        # GOAL STATE

        print()
        print("Goal state")
        print()

        grid_goal = get_random_free_point(grid_ground)
        local_goal = grid_to_local(grid_goal, (-1)*DRONE_MAX_ALTITUDE, north_min, east_min, CELL_SIZE)   
        global_goal = local_to_global(local_goal, global_home)

        print("global frame ", global_goal)
        print("local_frame  ", local_goal)
        print("grid_frame   ", grid_goal)  
        
        # STATE SPACE

        print()
        print("State space")
        print()

        print("Deriving median axes...")
        median_axes = get_median_axes(grid_ground)

        print("Random sampling locations...")
        idx = np.random.uniform(0,median_axes.shape[0],N_STATES)
        idx = np.array(idx, dtype=int)
        state_space = set( [ tuple(state) for state in median_axes[idx,:] ] )

        print("Adding start / goal states to state space...")
        state_space.add(grid_start)
        state_space.add(grid_goal)

        # SEARCH FOR A PLAN

        print()
        print("Searching")
        print()
        time_before = time.time()
        print("Running RRT algorithm...")
        path_raw, g = rrt(state_space, grid_start, grid_goal, grid_max, MAX_SEARCH_DISTANCE)   
        print("Calculation time: ", time.time() - time_before) 

        print("Optimizing path...")
        path = get_optimized_path(path_raw, grid_max)

        print("Drawing plan map...")

        # Notes  1 - A 2D map showing the plan will be displayed as a sanity check.
        #        2 - This map will temporarily freeze the simulation.
        #        3 - Just close the map window to unfreeze the simulation.

        show_map(grid_ground,
                 grid_home = grid_home,
                 grid_start = grid_start,
                 grid_goal = grid_goal,
                 median_axes = None,
                 state_space = None,
                 graph = None,
                 path = path,
                 altitude = None,
                 title="Path (Optimized)")

        # RUN THE PLAN

        print()
        print("Waypoints")
        print()

        print("Deriving waypoints from path...")
        self.waypoints = get_waypoints_from_path(path, DRONE_MAX_ALTITUDE, north_min, east_min, CELL_SIZE)

        self.send_waypoints_to_sim()

        self.plan_ready = True


    def takeoff_transition(self):

        print()
        print("----- TAKEOFF PHASE ---")
        self.flight_phase = FlightPhases.TAKEOFF    

        print()
        print("Taking off...")
        self.target_position[2] = (-1)*DRONE_MAX_ALTITUDE    
        self.takeoff(DRONE_MAX_ALTITUDE)


    def waypoint_transition(self):

        print()
        print("----- WAYPOINT PHASE ---")
        self.flight_phase = FlightPhases.WAYPOINT        
        next_waypoint = self.waypoints[0,:]

        print()
        print("Next waypoint: ", next_waypoint)
        self.waypoints = np.delete(self.waypoints,0,axis=0)

        self.target_position[0] = next_waypoint[0]
        self.target_position[1] = next_waypoint[1]
        # target_position -> local NED frame -> altitude negative up
        # waypoint        -> local NEU frame -> altitude positive up
        self.target_position[2] = (-1) * next_waypoint[2] 
        # Ignores waypoint 4th item, which is heading (only required by cmd_position)

        self.cmd_position(*next_waypoint)


    def landing_transition(self):

        print()
        print("----- LANDING PHASE -----")
        self.flight_phase = FlightPhases.LANDING

        print()
        print("Landing...")
        self.target_position[2] = 0.0

        # Using cmd_position() instead of land()
        # because drone may overshoot landing location with land().
        # If the drone overshoots, the reached_target() method
        # won't work...

        self.cmd_position(int(self.target_position[0]),
                          int(self.target_position[1]),
                          int(self.target_position[2]),
                          0)


    def disarming_transition(self):

        print()
        print("----- DISARMING PHASE -----")
        self.flight_phase = FlightPhases.DISARMING

        print()
        print("Disarming motors...")
        self.disarm()

        print()
        print("Releasing control...")
        self.release_control()


    def manual_transition(self):

        print()
        print("----- MANUAL PHASE -----")  
        print()  
        self.flight_phase = FlightPhases.MANUAL
        self.stop()
        self.in_mission = False
        print("Mission completed !")


    # ----- UTILITIES ----- #

    def send_waypoints_to_sim(self):

        l = self.waypoints.tolist()
        n = len(l)
        print("Sending ", n, " waypoints to simulator...")

        data = msgpack.dumps(l)
        self.connection._master.write(data)
 

    def reached_target(self) -> bool:
        """Checks if drone has reached its target position."""

        d = np.linalg.norm(self.target_position - self.local_position)

        if d <= DEADBAND:

            print()
            print("reached target position")
            print("current (local frame): ", self.local_position)
            print("target (local frame):", self.target_position)
            return True

        else:

            return False    


# ----- MAIN ----- #

if __name__ == "__main__":

    # Establich connection with simulator
    protocol_ip_port = "tcp:" + IP + ":" + PORT
    conn = MavlinkConnection(protocol_ip_port, timeout=60)

    # Initialize drone
    drone = AutonomousDrone(conn)    
    drone.start()

