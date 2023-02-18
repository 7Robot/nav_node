import rospy
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import RobotData, Pic_Action
from virtual_robot.msg import Virtual_Robot_ActionAction, Virtual_Robot_ActionGoal
from std_msgs.msg import Bool
import numpy as np
import time
import cv2
import actionlib

import matplotlib.pyplot as plt

class NavigationNode():
    """
    Node permettant de gérer la navigation du robot, par évitement d'obstacles

    Attributs
    ---------
        - graph : networkx.Graph\n
            Graphe contenant les noeuds
        - robot_data : RobotData\n
            Dernier message reçu par le topic "robot_data"
        - position_goal : Point\n
            Dernier message reçu par le topic "position_goal"
        - obstacles : Obstacles\n
            Dernier message reçu par le topic "obstacles"
        - action_orders_pub : rospy.Publisher\n
            Publisher pour envoyer les ordres d'action au robot
        - cv_map : np.array\n
            Image de la carte de l'environnement
        - avoidance_mode : str\n
            Mode d'évitement des obstacles
        - avoidance_trigger_distance : float\n
            Distance à partir de laquelle l'évitement se déclenche
        - emergency_stop_distance : float\n
            Distance à partir de laquelle l'évitement d'urgence se déclenche
        - robot_x_dimension : float\n
            Dimension en x du robot
        - robot_y_dimension : float\n
            Dimension en y du robot
    
    Méthodes
    --------
        - find_closest_node(start_pos, graph, exclude=[])\n
            Trouve le noeud le plus proche de la position "start_pos"
        - robot_data_callback(msg)\n
            Callback pour récupérer les données du robot
        - obstacles_callback(msg)\n
            Callback pour récupérer les obstacles
        - position_goal_callback(msg)\n
            Callback pour récupérer la position cible
        - verify_obstacles(start_pos, end_pos, exclude=None)\n
            Vérifie si les obstacles entrent en collision avec le robot
        - find_path(start_pos, end_pos, exclude=None)\n
            Trouve le chemin le plus court entre les deux positions
    """
    def __init__(self, avoidance_mode, avoidance_trigger_distance,
                 emergency_stop_distance, robot_x_dimension, robot_y_dimension, graph_file, action_orders_pub, cv_obstacle_file, discretisation = 5):
        self.discretisation = discretisation
        self.path_without_obstacles = []
        self.action_orders_pub = action_orders_pub
        self.next_point = Point()


        #Map des obstacles fixes
        self.cvmap = self.__init_Cv(cv_obstacle_file)
        self.static_obstacles = []

        for i in range(self.cvmap.shape[0]//self.discretisation):
            for j in range(self.cvmap.shape[1]//self.discretisation):
                for ip in range(self.discretisation):
                    for jp in range(self.discretisation):
                        if self.cvmap[i*self.discretisation+ip][j*self.discretisation+jp] == 255:
                            if (i, j) not in self.static_obstacles:
                                self.static_obstacles.append((j, i))
                                break

        for i in range(-1,self.cvmap.shape[0]//self.discretisation+1):
            self.static_obstacles.append((-1, i))
            self.static_obstacles.append((self.cvmap.shape[1]//self.discretisation, i))
        
        for j in range(-1,self.cvmap.shape[1]//self.discretisation+1):
            self.static_obstacles.append((j, -1))
            self.static_obstacles.append((j,self.cvmap.shape[0]//self.discretisation))

        print("Done init static obstacles")
        np.save("static_obstacles.npy", self.static_obstacles)

        self.obstacles = self.static_obstacles

        self.dynamic_obstacles = []

        self.x_range = self.cvmap.shape[1]//self.discretisation+1
        self.y_range = self.cvmap.shape[0]//self.discretisation+1

        print("x_range : ", self.x_range)
        print("y_range : ", self.y_range)

        #Dstar
        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()
        self.k = dict()
        self.position_goal = None
        self.visited = set()
        self.u_set = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]

        self.log = []
        self.path = None

        self.real_path = []



    def __init_Cv(self, filename):
        """
        Initialise la carte de l'environnement (Private method)
        """
        MAP_PRECISION = 1  # cm/pixel
        # le rayon du robot (celui est assimilé a un cercle pour les collisions )
        ROBOT_RADIUS = 15  # cm
        # distance a partir de laquelle l'évitement se déclenche
        # cette valeur n'est utilisée que pour savoir à quel moment il faut désactiver l'évitement
        DIST_EVITEMENT = 30  # cm
        
        img = cv2.imread(filename)

        self.map_width = img.shape[1]/MAP_PRECISION
        self.map_height = img.shape[0]/MAP_PRECISION

        maps = {
            "hard_obstacle_map": {"data": img[:, :, 0], "dilate": ROBOT_RADIUS},
            "item_obstacle_map": {"data": img[:, :, 2], "dilate": ROBOT_RADIUS},
            "disable_evitement_map": {"data": img[:, :, 0], "dilate": DIST_EVITEMENT}
        }
        
        for m in maps.values():
            if m["dilate"] is not None:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (int(m["dilate"] / MAP_PRECISION), int(m["dilate"] / MAP_PRECISION)))
                m["data"] = cv2.dilate(m["data"], kernel, iterations=1)
        
        return maps["hard_obstacle_map"]["data"]

    def find_path_without_obstacles(self, start_pos, end_pos):
        """
        Trouve le chemin le plus court entre les deux positions
        sans prendre en compte les collisions avec les obstacles
        """
        if self.position_goal in self.obstacles:
            return None
        self.init_dstar()
        self.insert_dstar(self.position_goal, 0.0)
        i=0
        while True:
            result = self.process_state()
            if result == -1:
                raise Exception("No path found")
            i+=1            
            if self.t[start_pos] == 'CLOSED':
                break
        
        self.path_without_obstacles = self.rebuild_path(start_pos, end_pos)
        self.path = self.path_without_obstacles

        print("Found path in ", i, " iterations")
        return self.path_without_obstacles
        
        
    # Dstar related methods
    def init_dstar(self):
        for i in range(self.x_range):
            for j in range(self.y_range):
                self.t[(i, j)] = 'NEW'
                self.k[(i, j)] = 0.0
                self.h[(i, j)] = float("inf")
                self.PARENT[(i, j)] = None

        self.h[self.position_goal] = 0.0
        self.visited = set()
   
    def rebuild_path(self, s_start, s_end):
        """
        Find the path from s_start to s_end
        :param s_start: start node
        :param s_end: end node
        :return: list of nodes
        """

        path = [s_start]
        s = s_start
        while True:
            s = self.PARENT[s]
            path.append(s)
            if s == s_end:
                return path

    def process_state(self):
        s = self.min_state()  # get node in OPEN set with min k value
        self.log += [s]
        self.visited.add(s)

        if s is None:
            return -1  # OPEN set is empty

        k_old = self.get_k_min()  # record the min k value of this iteration (min path cost)
        self.delete(s)  # move state s from OPEN set to CLOSED set

        # k_min < h[s] --> s: RAISE state (increased cost)
        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.h[s_n] <= k_old and \
                        self.h[s] > self.h[s_n] + self.cost(s_n, s):

                    # update h_value and choose parent
                    self.PARENT[s] = s_n
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)

        # s: k_min >= h[s] -- > s: LOWER state (cost reductions)
        if k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                        (self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    # 3) s_n find a better parent
                    self.PARENT[s_n] = s
                    self.insert_dstar(s_n, self.h[s] + self.cost(s, s_n))
        else:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    self.PARENT[s_n] = s
                    self.insert_dstar(s_n, self.h[s] + self.cost(s, s_n))
                else:
                    if self.PARENT[s_n] != s and \
                            self.h[s_n] > self.h[s] + self.cost(s, s_n):

                        # Condition: LOWER happened in OPEN set (s), s should be explored again
                        self.insert_dstar(s, self.h[s])
                    else:
                        if self.PARENT[s_n] != s and \
                                self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                                self.t[s_n] == 'CLOSED' and \
                                self.h[s_n] > k_old:

                            # Condition: LOWER happened in CLOSED set (s_n), s_n should be explored again
                            self.insert_dstar(s_n, self.h[s_n])

        return self.get_k_min()

    def delete(self, s):
        """
        delete: move state s from OPEN set to CLOSED set.
        :param s: state should be deleted
        """
        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'

        self.OPEN.remove(s)

    def get_k_min(self):
        """
        calc the min k value for nodes in OPEN set.
        :return: k value
        """

        if not self.OPEN:
            return -1

        return min([self.h[x] for x in self.OPEN])
    
    def get_neighbor(self, s):
        nei_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obstacles:
                nei_list.add(s_next)
                print("From "+str(s)+", "+str(s_next))

        return nei_list

    def min_state(self):
        """
        choose the node with the minimum k value in OPEN set.
        :return: state
        """

        if not self.OPEN:
            return None

        return min(self.OPEN, key=lambda x: self.k[x])

    def insert_dstar(self, s, h_new):
        """
        insert node into OPEN set.
        :param s: node
        :param h_new: new or better cost to come value
        """

        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)

        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return np.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])
    
    def is_collision(self, s_start, s_end):
        if s_start in self.obstacles or s_end in self.obstacles:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obstacles or s2 in self.obstacles:
                return True

        return False
    
    def adapt_path(self, new_obstacles):
        pass


    # Callbacks

    def position_goal_callback(self, msg):
        """
        Callback pour récupérer la position cible
        """
        print("msg: " + str(msg))
        msg = (msg.x*100//self.discretisation, msg.y*100//self.discretisation)
        if msg != self.position_goal:
            # Position goal has changed
            self.position_goal = msg
    
            pos = (self.robot_data.position.x*100//self.discretisation, self.robot_data.position.y*100//self.discretisation)
            print("Position goal: " + str(self.position_goal))
            print("Position robot: " + str(pos))
            self.init_dstar()
            t = time.time()
            path = self.find_path_without_obstacles(pos, self.position_goal)
            print("Path found in " + str(time.time() - t) + "s")
            if path != None:
                rospy.loginfo('Path found: ' + str(path))
                self.publish_pic_msg(path)
                self.next_point
            else:
                rospy.loginfo('No path found')
        
    def publish_pic_msg(self, path):
        """
        Publie un message ordonnant au robot de suivre le chemin "path"

        Paramètres
        ----------
            - path : liste\n
                Liste ordonnée des noeuds constituant le chemin
        """
        msg = Pic_Action()
        msg.action_destination = 'motor'
        msg.action_msg = 'CHAINEDMOVE'
        for node in path[1:-1]:
            msg.action_msg += ' ' + str(node[0]*self.discretisation/100) + ' ' + str(node[1]*self.discretisation/100)
        msg.action_msg += ' ' + str(self.position_goal[0]*self.discretisation/100) + ' ' + str(self.position_goal[1]*self.discretisation/100)
        self.action_orders_pub.publish(msg)

        if debug_mode:
            action_client_goal = Virtual_Robot_ActionGoal()
            action_client_goal.command = msg.action_msg
            action_client.send_goal(action_client_goal)
        
    def robot_data_callback(self, msg):
        self.robot_data = msg

    def obstacles_callback(self, msg):
        """
        Callback pour récupérer les obstacles

        converted_obstacles est une liste de la forme [x,y,radius,[liste de points...]]
        """


        converted_obstacles = [] #Liste de tuples (x,y,radius)

        tmp = [(obstacle.center.x, obstacle.center.y, obstacle.radius) for obstacle in msg.circles]

        # Convert data to a list of tuples
        for circ in tmp:
            converted_obstacles+=[[circ[0], circ[1], circ[2], []]]
            for i in range(0, 360):
                converted_obstacles[-1][3] +=  [[int(circ[0]*100 + circ[2]*np.cos(i*np.pi/180)*100)//self.discretisation, int(circ[1]*100 + circ[2]*np.sin(i*np.pi/180)*100)//self.discretisation]]
        
        #Find new obstacles

        new_obstacles = [converted_obstacles[k] for k in range(len(converted_obstacles))]
        for obs in converted_obstacles:
            for p in self.dynamic_obstacles:
                if obs[0]==p[0] and obs[1]==p[1] and obs[2]==p[2]:
                    # There is a new obstacle
                    new_obstacles.remove(p)
        
        if new_obstacles != []:
            # There are new obstacles
            self.dynamic_obstacles = converted_obstacles
            self.obstacles = self.static_obstacles + self.dynamic_obstacles
            print("There is new obstacles")
            
            # Adapt path



        # Is there any relevant removed obstacles ?
        for obs in self.dynamic_obstacles:
            removed = True
            for p in converted_obstacles:
                if obs[0]==p[0] and obs[1]==p[1] and obs[2]==p[2]:
                    removed = False
            if removed:
                # There is a removed obstacle
                print("There is removed obstacles")
                self.dynamic_obstacles = converted_obstacles
                self.obstacles = self.static_obstacles + self.dynamic_obstacles
                
                self.find_path_without_obstacles((self.robot_data.position.x*100//self.discretisation, self.robot_data.position.y*100//self.discretisation), self.position_goal)
                break


        
        if self.path != None:
            print("Callback")
            self.update_obstacles()
        
    
    def next_point_callback(self, msg):
        self.next_point = msg
    
    def motion_done_callback(self, msg):
        pass
        
                




if __name__ == "__main__":
    rospy.init_node('navigation_node', anonymous=False)

    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_x/Pos_goal')
    obstacles_topic = rospy.get_param('~obstacles_topic', '/obstacles')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_x/action')
    next_point_topic = rospy.get_param('~next_point_topic', '/robot_x/asserv_next_point')
    graph_file = rospy.get_param('~graph_file', 'default')
    cv_obstacle_file = rospy.get_param('~cv_obstacle_file', 'default')
    debug_mode = rospy.get_param('~debug_mode', False)
    avoidance_mode = rospy.get_param('~avoidance_mode', 'default')
    avoidance_trigger_distance = rospy.get_param('~avoidance_trigger_distance', 0.3) #m
    emergency_stop_distance = rospy.get_param('~emergency_stop_distance', 0.2) #m
    robot_data_topic = rospy.get_param('~robot_data_topic', '/robot_x/Odom')
    robot_x_dimension = rospy.get_param('~robot_x_dimension', 0.5) #m
    robot_y_dimension = rospy.get_param('~robot_y_dimension', 0.2) #m

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=1)

    if debug_mode:
        action_client = actionlib.SimpleActionClient('pic_action', Virtual_Robot_ActionAction)

    # Création de la classe NavigationNode
    Nav_node = NavigationNode(avoidance_mode, avoidance_trigger_distance, emergency_stop_distance,
                              robot_x_dimension, robot_y_dimension, graph_file, action_orders_pub, cv_obstacle_file)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(robot_data_topic, RobotData, Nav_node.robot_data_callback)
    rospy.Subscriber(obstacles_topic, Obstacles, Nav_node.obstacles_callback)
    rospy.Subscriber(next_point_topic, Point, Nav_node.next_point_callback)
    rospy.Subscriber('/robot_x/motion_done', Bool, Nav_node.motion_done_callback)

    # Vérification de la présence d'obstacle sur le chemin du robot
    while not rospy.is_shutdown():

        rospy.sleep(0.05)