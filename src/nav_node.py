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

        maps = {
            "hard_obstacle_map": {"data": img[:, :, 0], "dilate": ROBOT_RADIUS},
            "item_obstacle_map": {"data": img[:, :, 2], "dilate": ROBOT_RADIUS},
            "disable_evitement_map": {"data": img[:, :, 0], "dilate": DIST_EVITEMENT}
        }

        plt.imshow(maps["item_obstacle_map"]["data"])
        
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
        a = (end_pos[1] - start_pos[1]) / (end_pos[0] - start_pos[0])
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

    def find_path(self, graph, start_point, end_point, method='shortest_path'):
        """
        Trouve le chemin le plus court reliant la position "start_point" à la position "end_point"

        Paramètres
        ----------
            - start_point : tuple (x, y)\n
                Position de départ du chemin
            - end_point : tuple (x, y)\n
                Position d'arrivée du chemin
            - trigger_distance : float\n
                Distance à partir de laquelle on considère que le robot est dans une zone dangereuse

        Retourne
        --------
            - path : liste ordonnée de l'id des noeuds constituant le chemin
        """
        start = time.time()
        if method == 'closest_node':
            # On cherche tout d'abord le noeud le plus proche du robot
            start_node = self.find_closest_node(start_point, graph, trigger_distance=0.2)
            end_node = self.find_closest_node(end_point, graph, trigger_distance=0.2)
        elif method == 'shortest_path':
            graph = self.add_temporary_nodeandedge(start_point, end_point, graph)
            start_node = "temp_start_node"
            end_node = "temp_end_node"

        # On cherche le chemin le plus court entre le noeud le plus proche du robot et le noeud le plus proche de la position cible
        try:
            self.graph_modified = graph
            path = nx.astar_path(graph, start_node, end_node, heuristic=None, weight='weight')
        except nx.NetworkXNoPath:
            path = None
        rospy.loginfo('Time to find shortest path: ' + str(time.time() - start))
        return path

    def position_goal_callback(self, msg):
        """
        Callback permettant de récupérer la position cible

        Paramètres
        ----------
            - msg : Point\n
                Message contenant la position cible
        """
        self.position_goal = msg

        path = self.find_path(self.graph, self.robot_data.position, self.position_goal)
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
            msg.action_msg += ' ' + str(self.graph.nodes[node]["x"]) + ' ' + str(self.graph.nodes[node]["y"])
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

    def heuristic_function(Nav_Node , a, b):
        G = Nav_Node.graph_modified
        (x1, y1) = (G.nodes[a]['x'], G.nodes[a]['y'])
        (x2, y2) = (G.nodes[b]['x'], G.nodes[b]['y'])
        
        result = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

        return result


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
            