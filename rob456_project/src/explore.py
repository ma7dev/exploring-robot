#!/usr/bin/env python

# import relevant libraries
from __future__ import print_function

import roslib
import rospy
import math
import time

import numpy as np
import tf
from tf.transformations import euler_from_quaternion

# The geometry_msgs Twist message
from geometry_msgs.msg import Twist

# The move_base result message
from move_base_msgs.msg import MoveBaseActionResult

from actionlib_msgs.msg import GoalID

from nav_msgs.msg import OccupancyGrid, MapMetaData

# The odometry message
from nav_msgs.msg import Odometry

# Waypoint message for path_reset and path_ready topics
from std_msgs.msg import Empty

import utils

from AStarSearch import a_star_connection

MAP = np.zeros((4000,4000)) - 1

# how the map looked when we chose our last waypoint
OLD_MAP = None

# last time we copied MAP into OLD_MAP
last_refreshed = 0

# Number of meters per grid cell
RESOLUTION = 0

# current position in the occupancy grid
currentRow = 0 
currentCol = 0

# current coordinates of the robot
currentX = 0
currentY = 0
currentAngle = 0

# current coordinates of the goal
goalX = 0
goalY = 0

# stores all locations that we have visited
visited = np.zeros((4000, 4000))
# specify the radius around the robot that is visited 
# when it moves into a new cell
visit_width = 30

# radius of cells around candidate point that should not contain walls
avoidance_radius = 20

# radius to use to check for candidate points
window_size = 30

# keeps track of last location moved to in order to
# check if we are sitting still
last_location = (0,0)
last_location_time = 0
in_place = False
sitting_time = 8

# used to keep track of if we are currently calculating the next waypoint
thinking = False

# determine the next waypoint
def get_best_waypoint(MAP, current_pos, thresh=np.inf):
  global visited, window_size, currentCol, currentCol
  print('-'*60)
  print("current position:", utils.row_col_to_coord(current_pos, RESOLUTION, MAP))

  # total number of cells in the search window
  area = (2*window_size + 1) ** 2
  
  t0 = time.time()

  # compute candidate waypoints. These candidates are a list of tuples (cost, coordinate) where 
  # cost = g(x) * h(x) where
  # g(x) is the sum of all elements within the sliding window PLUS the number of cells within the window
  # h(x) is the euclidean distance between the center of the sliding window and the robots current position
  # candidates are only added if the region around the center of the window does not contain walls
  # AND the region around the center of the window is empty
  l = [((MAP[i-window_size:i+window_size+1, j-window_size:j+window_size+1].sum()+area) * utils.euclid_dist(current_pos, (i,j)), (i,j)) \
       for i in range(window_size, len(MAP)-window_size, window_size//2) \
       for j in range(window_size, len(MAP[i])-window_size, window_size//2) \
       if not (visited[i-avoidance_radius:i+avoidance_radius+1, j-avoidance_radius:j+avoidance_radius+1] == 1).any() \
          and (MAP[i-2:i+3, j-2:j+3] == 0).all() ]

  t1 = time.time()
  print("Time to generate candidates waypoints:", round(t1-t0, 2), "seconds")
  
  l = sorted(l, reverse=True)
  best_loc = l.pop()

  while not reachable((currentRow, currentCol), best_loc[1], MAP):
    print(best_loc, "is unreachable")
    if len(l) == 0:
      print("NO SOLUTIONS FOUND")
      return None
    # current position is unexplored, so no other cell will be considered reachable 
    # so tell the robot to move to closest explored cell
    if(MAP[currentRow][currentCol] == -1):
      i=1
      stepsize=10
      a=1
      x=currentCol
      y=currentRow
      done =0
      while(done != 1):
        for b in range(1, a*stepsize+1, stepsize):
          x+=stepsize*b*i
          if (MAP[x][y] == 0):
            done=1
            break
        if(done == 1):
          break
        for b in range(1, a*stepsize+1, stepsize):
          y+=stepsize*b*i
          if (MAP[x][y] == 0):
            done=1
            break

        i=i*-1
        a+=1
      best_loc = (0,(x, y))
      break
    best_loc = l.pop()

  t2 = time.time()
  print("Time to validate waypoint:", round(t2-t1, 2), "seconds")
  print("Total time:", round(t2-t0, 2), "seconds")
  print("Best position:", best_loc)
  
  best_loc = best_loc[1]
  print("Unsearched area in region:", np.count_nonzero(MAP[best_loc[0]-window_size:best_loc[0]+window_size+1, best_loc[1]-window_size:best_loc[1]+window_size+1] == -1))
  return utils.row_col_to_coord(best_loc, RESOLUTION, MAP)

# function to clear any old waypoints and publish the next waypoint
def go_to_next_waypoint():
  global OLD_MAP, MAP, currentX, currentY, currentAngle, goalX, goalY, thinking
  print("Calculating next waypoint...")
  thinking = True
  OLD_MAP = MAP.copy()
  clear_pub.publish(Empty())

  print("Currently at position", int(currentX), int(currentY))

  goalX, goalY = get_best_waypoint(MAP, current_pos=(currentRow, currentCol))

  composition_mat = utils.get_global_to_local_transform(currentX, currentY, currentAngle)
   
  goal_mat = np.dot(composition_mat, np.array([[goalX], [goalY], [1]]))

  # Make a new Twist waypoint message
  waypoint = Twist()
  
  waypoint.linear.x = goal_mat[0][0]
  waypoint.linear.y = goal_mat[1][0]
  waypoint.linear.z = 0.0
  
  # Don't rotate the waypoint
  waypoint.angular.x = 0.0
  waypoint.angular.y = 0.0
  waypoint.angular.z = 0.0

  #show_world(MAP)
  waypoint_pub.publish(waypoint)
  ready_pub.publish(Empty())
  thinking = False
  
# returns true if there is a path from current_loc to end_loc
def reachable(current_loc, end_loc, MAP):
  cost, path = a_star_connection(MAP, current_loc, end_loc)
  return cost[end_loc] != np.inf

# tell the robot to back up
def backup(amount):
  print("Backing up...")
  twist = Twist()
  twist.linear.x = -amount
  vel_pub.publish(twist)

# returns true if the robot hasn't moved in sitting_time seconds
def sitting_still():
  global currentRow, currentCol, in_place, last_location, last_location_time, sitting_time
  if (currentRow, currentCol) == last_location:
    if not in_place:
      in_place = True
      last_location_time = time.time()
    if time.time() - last_location_time > sitting_time:
      return True
      
  else:
    last_location = (currentRow, currentCol)
    in_place = False

  return False

# tell the robot to spin 360 degrees
def spin_in_place():
  for i in range(4):
    # Make a new Twist waypoint message
    waypoint = Twist()
    waypoint_pub.publish(waypoint)
    # Make a new Twist waypoint message
    waypoint = Twist()
    
    waypoint.linear.x = 0.0
    waypoint.linear.y = 0.0
    waypoint.linear.z = 0.0
    
    # Don't rotate the waypoint
    waypoint.angular.x = 0.0
    waypoint.angular.y = 0.0
    waypoint.angular.z = 90.0

    #show_world(MAP)
    waypoint_pub.publish(waypoint)
  ready_pub.publish(Empty())

# gets called when we reach a waypoint
def mb_callback(msg):
  # Check if robot has reached goal
  if msg.status.status == 2 or msg.status.status == 4 or msg.status.status == 5 or msg.status.status == 6:
    clear_pub.publish(Empty())
    backup(400)
    spin_in_place()
    print("Robot failed to reach waypoint!")
  elif msg.status.status == 3:
    print("Robot successfully reached waypoint!")

  go_to_next_waypoint()
  
# gets called every time the map updates
def map_callback(msg):
  global MAP, RESOLUTION
  RESOLUTION = msg.info.resolution
  MAP = utils.occupancygrid_to_numpy(msg)

def odom_callback(msg):
  global OLD_MAP, currentX, currentY, currentAngle, last_refreshed, visited, currentRow, currentCol, thinking

  # find current (x,y) position of robot based on odometry
  currentX = msg.pose.pose.position.x
  currentY = msg.pose.pose.position.y

  # find current orientation of robot based on odometry (quaternion coordinates)
  xOr = msg.pose.pose.orientation.x
  yOr = msg.pose.pose.orientation.y
  zOr = msg.pose.pose.orientation.z
  wOr = msg.pose.pose.orientation.w

  # find orientation of robot (Euler coordinates)
  (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

  # find currentAngle of robot (equivalent to yaw)
  # now that you have yaw, the robot's pose is completely defined by (currentX, currentY, currentAngle)
  currentAngle = yaw

  if OLD_MAP is None:
    return

  # cancel the current waypoint and assign a new one if we aren't moving
  if not thinking and sitting_still():
    backup(400)
    clear_pub.publish(Empty())
    cancel_pub.publish(GoalID())

  currentRow, currentCol = utils.coord_to_row_col((currentX, currentY), RESOLUTION, MAP)

  # mark the area around the region we are in as visited
  visited[currentRow-visit_width:currentRow+visit_width, currentCol-visit_width:currentCol+visit_width] = 1

  # check if we are in a region that was unexplored when we
  # originally chose this waypoint. If so, the current
  # path might take us through a wall, so it is better to
  # cancel the waypoint and reroute.
  refresh_rate = 10
  if (OLD_MAP[currentRow-7:currentRow+8, currentCol-7:currentCol+8] == -1).all() \
     and time.time() - last_refreshed > refresh_rate:
    print("Reassessing route...")
    last_refreshed = time.time() 
    backup(400)
    clear_pub.publish(Empty())
    cancel_pub.publish(GoalID())

if __name__ == "__main__":
  # Initialize the node
  rospy.init_node('move_in_square')
  
  # Publish waypoint data to robot
  waypoint_pub = rospy.Publisher('/base_link_goal',Twist, queue_size=10)
  
  # Publish messages telling the robot to clear the waypoint queue
  clear_pub = rospy.Publisher('/path_reset', Empty, queue_size = 10)

  # Publish messages telling when to begin waypoint nevigation
  ready_pub = rospy.Publisher('/path_ready', Empty, queue_size = 10)

  # Publish actuator commands
  vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

  # Publish message telling the robot to stop navigation to the current waypoint
  cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size = 10)

  # Subscribe to move_base result
  result_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult, mb_callback)

  # Subscribe to move_base result
  map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
  
  # subscribe to odometry message    
  odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

  # Turn control over to ROS
  rospy.spin()
