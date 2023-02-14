from distutils.log import debug
from hashlib import new
import rospy
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import RobotData, Pic_Action
from virtual_robot.msg import Virtual_Robot_ActionAction, Virtual_Robot_ActionGoal
from std_msgs.msg import Bool
import networkx as nx
import numpy as np
import time
import cv2
import actionlib

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
                 emergency_stop_distance, robot_x_dimension, robot_y_dimension, graph_file, action_orders_pub, cv_obstacle_file):

        self.action_orders_pub = action_orders_pub
        # Recupération du fichier de graphe
        self.graph = nx.read_gml(graph_file)
        self.graph_modified = self.graph.copy()

        # Récupération de la carte des obstacles statiques
        self.cv_map = self.__init_Cv(cv_obstacle_file)

        # Réglage des distances d'évitement par défaut 
        self.avoidance_mode = avoidance_mode
        self.avoidance_trigger_distance = avoidance_trigger_distance
        self.emergency_stop_distance = emergency_stop_distance

        # Réglage des dimensions du robot
        self.robot_x_dimension = robot_x_dimension
        self.robot_y_dimension = robot_y_dimension

        self.alternaive_path_given = False

        # Initialisation des objets D_star
        self.discretisation=5 # cm

        self.dstar = DStar(w=int(self.map_width/self.discretisation), h=int(self.map_height/self.discretisation))
        for i in range(int(len(self.cv_map)/self.discretisation)):
            for j in range(int(len(self.cv_map[0])/self.discretisation)):
                if self.cv_map[i*self.discretisation, j*self.discretisation] != 0:
                    self.dstar.Env.add_obstacle((j, i))

        # Initialisation des Messages
        self.robot_data = RobotData()
        self.position_goal = Point()
        self.next_point = Point()
        self.obstacles = Obstacles()

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
        
        return maps["item_obstacle_map"]["data"]

    def find_closest_node(self, start_pos, graph, exclude=[], trigger_distance=0.5):
        """
        Trouve le noeud le plus proche de la position "start_pos"

        Paramètres
        ----------
            - start_pos : tuple (x, y)\n
                Position de départ du chemin
            - graph : networkx.Graph\n
                Graphe contenant les noeuds
            - exclude : list\n
                Liste des noeuds à exclure de la recherche
        
        Retourne
        --------
            - node : int\n
                Id du noeud le plus proche de la position "start_pos"
        """
        
        closest_node = None
        closest_distance = float('inf')
        nodes = graph.nodes()

        for node in exclude: nodes.remove_node(node)
        for node in nodes:
            distance = np.sqrt((start_pos.x - graph.nodes[node]["x"])**2 + (start_pos.y - graph.nodes[node]["y"])**2)
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node
        if self.verify_obstacles((start_pos.x, start_pos.y), (graph.nodes[closest_node]["x"], graph.nodes[closest_node]["y"]), exclude='start_and_end', trigger_distance=trigger_distance):
            node_to_exclude = exclude.append(node)
            print(node_to_exclude)
            closest_node = self.find_closest_node(start_pos, graph, exclude=exclude.append(node))
        return closest_node

    def add_temporary_nodeandedge(self, start_pos, end_pos, graph):
        """
        Ajoute un noeud temporaire et une arête entre le noeud le plus proche de la position "start_pos" et le noeud temporaire

        Paramètres
        ----------
            - start_pos : tuple (x, y)\n
                Position de départ du chemin
            - end_pos : tuple (x, y)\n
                Position d'arrivée du chemin
            - graph : networkx.Graph\n
                Graphe contenant les noeuds
        
        Retourne
        --------
            - graph : networkx.Graph\n
                Graphe contenant les noeuds
        """
        graph_modified = graph.copy()
        graph_modified.add_node("temp_end_node", x=end_pos.x, y=end_pos.y)
        graph_modified.add_node("temp_start_node", x=start_pos.x, y=start_pos.y)
        for node in graph_modified.nodes():
            if not self.verify_obstacles((graph_modified.nodes[node]["x"], graph_modified.nodes[node]["y"]), (end_pos.x, end_pos.y), exclude=['dynamic']):
                graph_modified.add_edge(node, "temp_end_node", weight=np.sqrt((graph_modified.nodes[node]["x"] - end_pos.x)**2 + (graph_modified.nodes[node]["y"] - end_pos.y)**2))
            
            if not self.verify_obstacles((graph_modified.nodes[node]["x"], graph_modified.nodes[node]["y"]), (start_pos.x, start_pos.y), exclude=['dynamic']):
                graph_modified.add_edge(node, "temp_start_node", weight=np.sqrt((graph_modified.nodes[node]["x"] - start_pos.x)**2 + (graph_modified.nodes[node]["y"] - start_pos.y)**2))
        return graph_modified
        
    def get_line_equation(self, start_pos, end_pos):
        """
        Retourne l'équation de la droite passant par les deux points

        Paramètres
        ----------
            - start_pos : tuple (x, y)\n
                Position de départ du chemin
            - end_pos : tuple (x, y)\n
                Position d'arrivée du chemin
        
        Retourne
        --------
            - a : float\n
                Coefficient directeur de la droite
            - b : float\n
                Ordonnée à l'origine de la droite
        """
        try:
            a = (end_pos[1] - start_pos[1]) / (end_pos[0] - start_pos[0])
        except:
            a = 0
        b = start_pos[1] - a * start_pos[0]
        return a, b
    
    def verify_obstacles(self, start_point, end_point, exclude=[], trigger_distance=None):
        """
        Vérifie si le segment reliant la position "start_point" à la position "end_point" traverse un obstacle statique (éléments fixes du plateau)
        ou dynamique (autres robots)

        Paramètres
        ----------
            - start_point : tuple (x, y)\n
                Position de départ du segment
            - end_point : tuple (x, y)\n
                Position d'arrivée du segment
            - exclude : list ou str\n
                Paramètre permettant d'exclure des obstacles de la vérification
                plusieurs paramètres sont possibles et peuvent être combinés :
                    - 'start_and_end' : ne considère pas le segement comme obstrué si il commence ou termine dans un obstacle
                    - 'static' : ne considère que les obstacles statiques (définis dans le fichier cv_obstacle_file)
                    - 'dynamic' : ne considère que les obstacles dynamiques renvoyés par le topic "obstacles"
            - trigger_distance : float\n
                Distance à laquelle un obstacle dynamique est considéré comme problématique

        Retourne
        --------
            - bool : True si le segment traverse un obstacle, False sinon
        """
        if trigger_distance is None: trigger_distance = self.avoidance_trigger_distance

        # Vérification de collisions avec des obstacles statiques
        if 'static' not in exclude:
            oks = []
            x1, y1 = int(start_point[0]*100), int(start_point[1]*100)
            x2, y2 = int(end_point[0]*100), int(end_point[1]*100)
            if y1 < y2 and x1 < x2:
                for alpha in np.arange(0., 1., 1 / max(abs(y1 - y2), abs(x1 - x2))) :
                    newx = int(x1 + alpha * (max(x1, x2) - min(x1, x2)))
                    newy = int(y1 + alpha * (max(y1, y2) - min(y1, y2)))
                    oks.append(self.cv_map[newy, newx])
            elif y1 > y2 and x1 < x2:
                for alpha in np.arange(0., 1., 1 / max(abs(y1 - y2), abs(x1 - x2))) :
                    newx = int(x1 + alpha * (max(x1, x2) - min(x1, x2)))
                    newy = int(y1 - alpha * (max(y1, y2) - min(y1, y2)))
                    oks.append(self.cv_map[newy, newx])
            elif y1 < y2 and x1 > x2:
                for alpha in np.arange(0., 1., 1 / max(abs(y1 - y2), abs(x1 - x2))) :
                    newx = int(x1 - alpha * (max(x1, x2) - min(x1, x2)))
                    newy = int(y1 + alpha * (max(y1, y2) - min(y1, y2)))
                    oks.append(self.cv_map[newy, newx])
            elif y1 > y2 and x1 > x2:
                for alpha in np.arange(0., 1., 1 / max(abs(y1 - y2), abs(x1 - x2))) :
                    newx = int(x1 - alpha * (max(x1, x2) - min(x1, x2)))
                    newy = int(y1 - alpha * (max(y1, y2) - min(y1, y2)))
                    oks.append(self.cv_map[newy, newx])
            # Si le point d'arrivé ou le point de départ se trouve dans un obstacle et que les point d'arrivé et de départs sont exclu
            if 'start_and_end' in exclude and (self.cv_map[y1, x1] != 0 or self.cv_map[y2, x2] != 0): return False 

            try:
                if np.max(oks) != 0:
                    return True
                else:
                    return None
            except:
                return None
        # Verifing collisions with dynamic obstacles
        elif exclude != 'dynamic':
            # Déterminons l'équation de la droite reliant le point de départ et d'arrivée
            m, o = self.get_line_equation(start_point, end_point)

            # On résoud maintenant l'équation pour trouver les points d'intersection entre cette droite et les cercles représentant les obstacles
            # L'équation à résoudre est la suivante : x^2 * (a^2 + 1) + x*(2*a*b - 2*xc - 2*a*yc) + b^2 + xc^2 + yc^2 - 2*yc*b - R^2 = 0
            
            for obstacle in self.obstacles.circles:
                xc = obstacle.center.x
                yc = obstacle.center.y
                R = trigger_distance

                # Calculs des coefficients de l'équation
                a = m**2 + 1
                b = 2*m*o - 2*xc - 2*m*yc
                c = xc**2 + yc**2 - 2*yc*o + o**2 - R**2
                # On résoud l'équation pour trouver les points d'intersection entre la droite et le cercle
                delta = b**2 - 4*a*c
                if delta > 0:
                    x1 = (-b + np.sqrt(delta)) / (2*a)
                    x2 = (-b - np.sqrt(delta)) / (2*a)
                    y1 = m*x1 + o
                    y2 = m*x2 + o
                    # On vérifie si les points d'intersection sont dans le segment
                    if x1 >= min(start_point[0], end_point[0]) and x1 <= max(start_point[0], end_point[0]) and y1 >= min(start_point[1], end_point[1]) and y1 <= max(start_point[1], end_point[1]):
                        return obstacle
                    elif x2 >= min(start_point[0], end_point[0]) and x2 <= max(start_point[0], end_point[0]) and y2 >= min(start_point[1], end_point[1]) and y2 <= max(start_point[1], end_point[1]):
                        return obstacle
            return None

    def rm_edges_around_obstacle(self, obstacle):
        """
        Supprime les arêtes du graphe qui traversent l'obstacle

        Paramètres
        ----------
            - obstacle : Circle\n
                Obstacle à prendre en compte
        
        Retourne
        --------
            - graph_modified : nx.Graph\n
                Graphe comprenant les modifications sur les arrêtes
        """
        graph_modified = self.graph.copy()
        for edge in self.graph.edges():
            start_node = (self.graph.nodes[edge[0]]['x'], self.graph.nodes[edge[0]]['y'])
            end_node = (self.graph.nodes[edge[1]]['x'], self.graph.nodes[edge[1]]['y'])
            if self.verify_obstacles(start_node, end_node, ['static']) != None:
                graph_modified.remove_edge(edge[0], edge[1])
        
        # On supprime les noeuds qui sont dans le cercle de l'obstacle
        for node in self.graph.nodes():
            if np.sqrt((self.graph.nodes[node]['x'] - obstacle.center.x)**2 + (self.graph.nodes[node]['y'] - obstacle.center.y)**2) < self.avoidance_trigger_distance:
                graph_modified.remove_node(node)
        return graph_modified

    def find_path(self, start_point, end_point):
        """
        Trouve le chemin le plus court entre deux points

        Paramètres
        ----------
            - start_point : Point\n
                Point de départ
            - end_point : Point\n
                Point d'arrivée
        
        Retourne
        --------
            - path : list\n
                Liste ordonnée des noeuds constituant le chemin dans un monde sans obstacle
        """
        s_start=(int(100*start_point.x/self.discretisation), int(100*start_point.y/self.discretisation))
        s_end=(int(100*end_point.x/self.discretisation), int(100*end_point.y/self.discretisation))

        (self.dstar.s_start, self.dstar.s_end) = (s_start, s_end)

        t = time.time()
        print("s_start : ", s_start)
        print("s_end : ", s_end)

        
        print("Start : ", t)
        self.dstar.init()
        self.dstar.insert(s_end, 0)

        while True:
            self.dstar.process_state()
            if self.dstar.t[s_start] == 'CLOSED':
                break
        

        print("End : ", time.time() - t)
        self.dstar.path_without_obstacle = self.dstar.extract_path(s_start, s_end)
        self.dstar.path_without_obstacle = self.convert_path(self.dstar)
        return self.calculate_path_with_obstacles()

    def convert_path(self, dstar):

        converted_path = []
        for point in dstar.path_without_obstacle:
            converted_path.append((self.discretisation*point[0]/100, self.discretisation*point[1]/100))
        
        return converted_path

    def position_goal_callback(self, msg):
        """
        Callback permettant de récupérer la position cible

        Paramètres
        ----------
            - msg : Point\n
                Message contenant la position cible
        """
        self.position_goal = msg

        path_without_obstacle = self.find_path(self.robot_data.position, self.position_goal)

        self.dstar.on_press()

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
            msg.action_msg += ' ' + str(node[0]) + ' ' + str(node[1])
        msg.action_msg += ' ' + str(self.position_goal.x) + ' ' + str(self.position_goal.y)
        self.action_orders_pub.publish(msg)

        if debug_mode:
            action_client_goal = Virtual_Robot_ActionGoal()
            action_client_goal.command = msg.action_msg
            action_client.send_goal(action_client_goal)

    def robot_data_callback(self, msg):
        self.robot_data = msg
    
    def obstacles_callback(self, msg):
        self.obstacles = msg

    def next_point_callback(self, msg):
        self.next_point = msg
    
    def motion_done_callback(self, msg):
        pass

    def calculate_path_with_obstacles(self):
        for obstacle in self.obstacles.circles:
            print("obstacle : ", (obstacle.center.x, obstacle.center.y))
            obs = (obstacle.center.x, obstacle.center.y)
            self.dstar.on_press(obs)
            print("ok")
        return self.dstar.path

class Env:
    def __init__(self, w, h):
        self.x_range = w  # size of background
        self.y_range = h
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = set()

    def add_obstacle(self, obs):
        self.obs.add(obs)

class DStar:
    def __init__(self, w, h, s_start=(0,0), s_goal=(0,0)):
        self.s_start, self.s_goal = s_start, s_goal

        self.Env = Env(w,h)

        self.u_set = self.Env.motions
        self.obs = self.Env.obs
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()
        self.k = dict()
        self.path_without_obstacle = []
        self.visited = set()
        self.count = 0

    def init(self):
        for i in range(self.Env.x_range):
            for j in range(self.Env.y_range):
                self.t[(i, j)] = 'NEW'
                self.k[(i, j)] = 0.0
                self.h[(i, j)] = float("inf")
                self.PARENT[(i, j)] = None

        self.h[self.s_goal] = 0.0

    def run(self):
        s_start, s_end = self.s_start, self.s_goal
        self.init()
        self.insert(s_end, 0)

        while True:
            self.process_state()
            if self.t[s_start] == 'CLOSED':
                break

        self.path = self.extract_path(s_start, s_end)

    def on_press(self, new_obstacle):
        x, y = new_obstacle
        x, y = int(x), int(y)
        if (x, y) not in self.obs:
            self.obs.add((x, y))

            s = self.s_start
            self.visited = set()
            self.count += 1

            while s != self.s_goal:
                if self.is_collision(s, self.PARENT[s]):
                    self.modify(s)
                    continue
                s = self.PARENT[s]

            self.path = self.extract_path(self.s_start, self.s_goal)

    def extract_path(self, s_start, s_end):
        path = [s_start]
        s = s_start
        while True:
            s = self.PARENT[s]
            path.append(s)
            if s == s_end:
                return path

    def process_state(self):
        s = self.min_state()  # get node in OPEN set with min k value
        self.visited.add(s)
        print("process state: ", s)

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
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
        else:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                        (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):

                    # Condition:
                    # 1) t[s_n] == 'NEW': not visited
                    # 2) s_n's parent: cost reduction
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
                else:
                    if self.PARENT[s_n] != s and \
                            self.h[s_n] > self.h[s] + self.cost(s, s_n):

                        # Condition: LOWER happened in OPEN set (s), s should be explored again
                        self.insert(s, self.h[s])
                    else:
                        if self.PARENT[s_n] != s and \
                                self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                                self.t[s_n] == 'CLOSED' and \
                                self.h[s_n] > k_old:

                            # Condition: LOWER happened in CLOSED set (s_n), s_n should be explored again
                            self.insert(s_n, self.h[s_n])

        return self.get_k_min()

    def min_state(self):
        """
        choose the node with the minimum k value in OPEN set.
        :return: state
        """

        if not self.OPEN:
            return None

        return min(self.OPEN, key=lambda x: self.k[x])

    def get_k_min(self):
        """
        calc the min k value for nodes in OPEN set.
        :return: k value
        """

        if not self.OPEN:
            return -1

        return min([self.k[x] for x in self.OPEN])

    def insert(self, s, h_new):
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

    def delete(self, s):
        """
        delete: move state s from OPEN set to CLOSED set.
        :param s: state should be deleted
        """

        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'

        self.OPEN.remove(s)

    def modify(self, s):
        """
        start processing from state s.
        :param s: is a node whose status is RAISE or LOWER.
        """

        self.modify_cost(s)

        while True:
            k_min = self.process_state()

            if k_min >= self.h[s]:
                break

    def modify_cost(self, s):
        # if node in CLOSED set, put it into OPEN set.
        # Since cost may be changed between s - s.parent, calc cost(s, s.p) again

        if self.t[s] == 'CLOSED':
            self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))

    def get_neighbor(self, s):
        nei_list = set()

        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

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
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False


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
        if avoidance_mode == 'default':
            if (Nav_node.robot_data.position.x, Nav_node.robot_data.position.y) != (Nav_node.next_point.x, Nav_node.next_point.y):
                start = time.time()

                # On calcule l'équation de droite reliant la position du robot au prochain point
                a, b = Nav_node.get_line_equation((Nav_node.robot_data.position.x, Nav_node.robot_data.position.y), (Nav_node.next_point.x, Nav_node.next_point.y))

                if np.sqrt((Nav_node.robot_data.position.x - Nav_node.next_point.x)**2 + (Nav_node.robot_data.position.y - Nav_node.next_point.y)**2) > Nav_node.avoidance_trigger_distance:
                    # On trouve le point situé à une distance "avoidance_trigger_distance" du robot
                    x_avoid = np.sqrt(avoidance_trigger_distance**2 / (1 + a**2)) + Nav_node.robot_data.position.x
                    y_avoid = a * x_avoid + b
                else:
                    x_avoid = Nav_node.next_point.x
                    y_avoid = Nav_node.next_point.y

                obstacle = Nav_node.verify_obstacles((Nav_node.robot_data.position.x, Nav_node.robot_data.position.y), (x_avoid, y_avoid), ['static'])
                if obstacle != None:
                    rospy.loginfo('New Obstacle detected at %.2f, %.2f' % (obstacle.center.x, obstacle.center.y))
                    graph_modified = Nav_node.rm_edges_around_obstacle(obstacle)

                    path = Nav_node.find_path(graph_modified, Nav_node.robot_data.position, Nav_node.position_goal, 'closest_node')
                    if path != None and Nav_node.alternaive_path_given == False:
                        rospy.loginfo('Alternative Path found: ' + str(path))
                        Nav_node.publish_pic_msg(path)
                        Nav_node.alternaive_path_given = True
                        rospy.wait_for_message("/robot_x/motion_done", Bool)
                    elif path == None:
                        rospy.loginfo('No alternative path found')
                else:
                    Nav_node.alternaive_path_given = False
        rospy.sleep(0.05)
            