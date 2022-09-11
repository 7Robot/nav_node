import rospy
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import RobotData, Pic_Action
import networkx as nx
import numpy as np
import time
import cv2

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
        self.graph = nx.read_gml(graph_file)

        self.cv_map = self.__init_Cv(cv_obstacle_file)

        self.avoidance_mode = avoidance_mode
        self.avoidance_trigger_distance = avoidance_trigger_distance
        self.emergency_stop_distance = emergency_stop_distance

        self.robot_x_dimension = robot_x_dimension
        self.robot_y_dimension = robot_y_dimension
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
        
        for m in maps.values():
            if m["dilate"] is not None:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (int(m["dilate"] / MAP_PRECISION), int(m["dilate"] / MAP_PRECISION)))
                m["data"] = cv2.dilate(m["data"], kernel, iterations=1)
        
        return maps["item_obstacle_map"]["data"]

    def find_closest_node(self, start_pos, graph, exclude=[]):
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
        for node in exclude: nodes.remove(node)
        for node in nodes:
            distance = np.sqrt((start_pos.x - graph.nodes[node]["x"])**2 + (start_pos.y - graph.nodes[node]["y"])**2)
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node
        if self.verify_obstacles((start_pos.x, start_pos.y), (graph.nodes[closest_node]["x"], graph.nodes[closest_node]["y"]), exclude='start_and_end'):
            closest_node = self.find_closest_node(self, start_pos, graph, exclude=exclude.append(node))
        return closest_node

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
    
    def verify_obstacles(self, start_point, end_point, exclude=[]):
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

        Retourne
        --------
            - bool : True si le segment traverse un obstacle, False sinon
        """
        # Verifing collisions with static obstacles
        if 'static' not in exclude:
            oks = []

            if start_point[1] < end_point[1] and start_point[0] < end_point[0]:
                for alpha in np.arange(0., 1., 1 / max(abs(start_point[1] - end_point[1]), abs(start_point[0] - end_point[0]))) :
                    newx = int(start_point[0] + alpha * (max(start_point[0], end_point[0]) - min(start_point[0], end_point[0])))
                    newy = int(start_point[1] + alpha * (max(start_point[1], end_point[1]) - min(start_point[1], end_point[1])))
                    oks.append(self.cv_map[newy, newx])
            elif start_point[1] > end_point[1] and start_point[0] < end_point[0]:
                for alpha in np.arange(0., 1., 1 / max(abs(start_point[1] - end_point[1]), abs(start_point[0] - end_point[0]))) :
                    newx = int(start_point[0] + alpha * (max(start_point[0], end_point[0]) - min(start_point[0], end_point[0])))
                    newy = int(start_point[1] - alpha * (max(start_point[1], end_point[1]) - min(start_point[1], end_point[1])))
                    oks.append(self.cv_map[newy, newx])
            elif start_point[1] < end_point[1] and start_point[0] > end_point[0]:
                for alpha in np.arange(0., 1., 1 / max(abs(start_point[1] - end_point[1]), abs(start_point[0] - end_point[0]))) :
                    newx = int(start_point[0] - alpha * (max(start_point[0], end_point[0]) - min(start_point[0], end_point[0])))
                    newy = int(start_point[1] + alpha * (max(start_point[1], end_point[1]) - min(start_point[1], end_point[1])))
                    oks.append(self.cv_map[newy, newx])
            elif start_point[1] > end_point[1] and start_point[0] > end_point[0]:
                for alpha in np.arange(0., 1., 1 / max(abs(start_point[1] - end_point[1]), abs(start_point[0] - end_point[0]))) :
                    newx = int(start_point[0] - alpha * (max(start_point[0], end_point[0]) - min(start_point[0], end_point[0])))
                    newy = int(start_point[1] - alpha * (max(start_point[1], end_point[1]) - min(start_point[1], end_point[1])))
                    oks.append(self.cv_map[newy, newx])
            # Si le point d'arrivé ou le point de départ se trouve dans un obstacle et que les point d'arrivé et de départs sont exclu
            if 'start_and_end' in exclude and (max(start_point) != 0 or max(end_point) != 0): return False 

            if np.max(oks) != 0:
                return True
            else:
                return None
        # Verifing collisions with dynamic obstacles
        elif exclude != 'dynamic':
            # Déterminons l'équation de la droite reliant le point de départ et d'arrivée
            o, m = self.get_line_equation(start_point, end_point)

            # On résoud maintenant l'équation pour trouver les points d'intersection entre cette droite et les cercles représentant les obstacles
            # L'équation à résoudre est la suivante : x^2 * (a^2 + 1) + x*(2*a*b - 2*xc - 2*a*yc) + b^2 + xc^2 + yc^2 - 2*yc*b - R^2 = 0
            
            for obstacle in self.obstacles.circles:
                xc = obstacle.center.x
                yc = obstacle.center.y
                R = obstacle.radius

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
                else:
                    return None


    def find_path(self, graph, start_point, end_point):
        """
        Trouve le chemin le plus court reliant la position "start_point" à la position "end_point"

        Paramètres
        ----------
            - start_point : tuple (x, y)\n
                Position de départ du chemin
            - end_point : tuple (x, y)\n
                Position d'arrivée du chemin

        Retourne
        --------
            - path : liste ordonnée de l'id des noeuds constituant le chemin
        """
        start = time.time()
        # On cherche tout d'abord le noeud le plus proche du robot
        start_node = self.find_closest_node(start_point, graph)
        end_node = self.find_closest_node(end_point, graph)

        # On cherche le chemin le plus court entre le noeud le plus proche du robot et le noeud le plus proche de la position cible
        path = nx.astar_path(graph, start_node, end_node)

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
        rospy.loginfo('Path found: ' + str(path))
        self.publish_pic_msg(path)
        
    
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
        for node in path:
            msg.action_msg += ' ' + str(self.graph.nodes[node]["x"]) + ' ' + str(-self.graph.nodes[node]["y"])
        msg.action_msg += ' ' + str(self.position_goal.x) + ' ' + str(-self.position_goal.y)
        self.action_orders_pub.publish(msg)

    def robot_data_callback(self, msg):
        self.robot_data = msg
    
    def obstacles_callback(self, msg):
        self.obstacles = msg

    def next_point_callback(self, msg):
        self.next_point = msg
        

if __name__ == "__main__":
    rospy.init_node('navigation_node', anonymous=False)

    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_x/Pos_goal')
    obstacles_topic = rospy.get_param('~obstacles_topic', '/obstacles')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_x/action')
    next_point_topic = rospy.get_param('~next_point_topic', '/robot_x/asserv_next_point')
    graph_file = rospy.get_param('~graph_file', 'default')
    cv_obstacle_file = rospy.get_param('~cv_obstacle_file', 'default')
    avoidance_mode = rospy.get_param('~avoidance_mode', 'default')
    avoidance_trigger_distance = rospy.get_param('~avoidance_trigger_distance', 0.5) #m
    emergency_stop_distance = rospy.get_param('~emergency_stop_distance', 0.5) #m
    robot_data_topic = rospy.get_param('~robot_data_topic', '/robot_x/Odom')
    robot_x_dimension = rospy.get_param('~robot_x_dimension', 0.5) #m
    robot_y_dimension = rospy.get_param('~robot_y_dimension', 0.2) #m

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=10)

    # Création de la classe NavigationNode
    Nav_node = NavigationNode(avoidance_mode, avoidance_trigger_distance, emergency_stop_distance,
                                robot_x_dimension, robot_y_dimension, graph_file, action_orders_pub, cv_obstacle_file)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(robot_data_topic, RobotData, Nav_node.robot_data_callback)
    rospy.Subscriber(obstacles_topic, Obstacles, Nav_node.obstacles_callback)
    rospy.Subscriber(next_point_topic, Point, Nav_node.next_point_callback)

    # Vérification de la présence d'obstacle sur le chemin du robot
    while not rospy.is_shutdown():
        if avoidance_mode == 'default':
            if (Nav_node.robot_data.position.x, Nav_node.robot_data.position.y) != (Nav_node.next_point.x, Nav_node.next_point.y):
                start = time.time()

                # On calcule l'équation de droite reliant la position du robot au prochain point
                a, b = Nav_node.get_line_equation(Nav_node.robot_data.position, Nav_node.next_point)

                if np.sqrt((Nav_node.robot_data.position.x - Nav_node.next_point.x)**2 + (Nav_node.robot_data.position.y - Nav_node.next_point.y)**2) > Nav_node.avoidance_trigger_distance:
                    # On trouve le point situé à une distance "avoidance_trigger_distance" du robot
                    x_avoid = np.sqrt(avoidance_trigger_distance**2 / (1 + a**2)) + Nav_node.robot_data.position.x
                    y_avoid = a * x_avoid + b
                else:
                    x_avoid = Nav_node.next_point.x
                    y_avoid = Nav_node.next_point.y

                obstacle = Nav_node.verify_obstacles((Nav_node.robot_data.position.x, Nav_node.robot_data.position.y), (x_avoid, y_avoid), ['static'])
                rospy.loginfo('Time to verify obstacles: ' + str(time.time() - start))
                if obstacle != None:
                    rospy.loginfo('Obstacle detected at %.2f, %.2f' % (obstacle.center.x, obstacle.center.y))
                    path = Nav_node.find_path(Nav_node.graph, Nav_node.robot_data.position, Nav_node.position_goal)
                    rospy.loginfo('Alternative Path found: ' + str(path))
                    Nav_node.publish_pic_msg(path)
        rospy.sleep(0.2)
            