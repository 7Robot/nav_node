import rospy
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import RobotData, Pic_Action
from virtual_robot.msg import Virtual_Robot_ActionAction, Virtual_Robot_ActionGoal
from std_msgs.msg import Bool
import numpy as np
import actionlib

class NavNode():
    def __init__(self, action_orders_pub, margin=0.03, pos = np.array([0,0]), max_radius = 0.35, max_iter = 15, distance_interpoint = 0.03, emergency_stop_distance = 0, simu_mode = False):
        self.path = []
        self.margin = margin
        self.position_goal = None
        self.max_iter = max_iter

        self.map_static_obstacles = np.load("FixedObstacleMap.npy")
        self.map_obstacles = self.map_static_obstacles.copy()
        self.max_radius = max_radius
        self.shape_board = self.map_obstacles.shape
        
        self.robot_data = RobotData()
        self.position = pos
        self.orientation = 0

        self.distance_interpoint = distance_interpoint
        self.next_goal = None
        self.emergency_stop_distance = emergency_stop_distance

        self.action_orders_pub = action_orders_pub
        self.simu_mode = False

    def find_middle_obstacles(self, path, path_portion : int):
        """
        Find the beginning and the end of the obstacle in the path portion

        Parameters
        ----------
        path_portion : int Portion de 

        """
        start = np.array(path[path_portion])
        end = np.array(path[path_portion + 1])

        discretisation = np.linspace(start, end, int(np.linalg.norm(end-start)/(np.linalg.norm(start - end)*0.01)))

        index_in_disc = 0
        (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])

        # Find the beginning of the obstacle
        while not(self.is_obstacle(x,y)) and index_in_disc < len(discretisation) - 1:
            index_in_disc += 1
            (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])
        
        if index_in_disc == len(discretisation) - 1:
            return False
        
        start_obstacle = discretisation[index_in_disc]

        # Find the end of the obstacle
        while self.is_obstacle(x,y) and index_in_disc < len(discretisation) - 1:
            index_in_disc += 1
            (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])
        
        if index_in_disc == len(discretisation) - 1:
            rospy.logwarn("Obstacle non fermé")
            return None
        else:
            end_obstacle = discretisation[index_in_disc]

            return (start_obstacle, end_obstacle)
        
    def is_obstacle(self, x, y):    # modifier pour que collision soit un encadrement et pas une valeur exacte ?
        """
        Vérifie si le point (x, y) touche un obstacle ou est sorti de la map
        """
        (xf,yf) = (int(x*100), int(y*100))
        shape = self.map_obstacles.shape
        if xf < 0 or yf < 0 or xf >= shape[0] or yf >= shape[1]:
            return True
        return self.map_obstacles[xf, yf] != 0

    def find_normal_non_obstacle(self, start, end):
        """ Find the nearest point outside of the obstacle space to define the new path
        Everything is in meter

        entry: 
            start: (x,y): start point of the obstacle
            end: (x,y): end point of the obstacle
        
        return:
            detour: (x,y): the nearest point outside of the obstacle space

        use: 
            self.map_obstacle: map
            self.margin: margin around the obstacle
        """

        vect_obstacle = np.array(end) - np.array(start)

        # Find the normal vector and normalize it with np.linalg.norm
        vec_norm = np.array([vect_obstacle[1], -vect_obstacle[0]])/np.linalg.norm(vect_obstacle) * 0.01

        # Find the middle point of the obstacle on the vector
        middle = np.array([(start[0]+end[0])/2, (start[1]+end[1])/2])

        middle_sup = middle.copy()
        middle_inf = middle.copy()

        # Max distance is 350 cm, don't want to go too far
        dist = 0

        # Find the nearest point outside of the obstacle space + margin
        while (self.is_obstacle(middle_sup[0],middle_sup[1]) and self.is_obstacle(middle_inf[0], middle_inf[1])) and dist < np.sqrt((self.shape_board[0]/100)**2 + (self.shape_board[1]/100)**2):
            middle_sup += vec_norm  
            middle_inf -= vec_norm    
            dist += 0.01

        # Return the nearest point
        if dist > 3.5:
            rospy.logwarn("Pas de point millieu trouvé")
            return None      # No point found
        else:
            if self.is_obstacle(middle_sup[0], middle_sup[1]):
                middle_inf_decal = middle_inf - self.margin * vec_norm * 100
                if not(self.is_obstacle(middle_inf_decal[0], middle_inf_decal[1])):
                    return middle_inf - self.margin * vec_norm * 100
                else:
                    rospy.logwarn("ATTENTION : Chemin étroit")
                    return middle_inf
            else:
                middle_sup_decal = middle_sup + self.margin * vec_norm * 100
                if not(self.is_obstacle(middle_sup_decal[0], middle_sup_decal[1])):
                    return middle_sup + self.margin * vec_norm * 100
                else:
                    rospy.logwarn("ATTENTION : Chemin étroit")
                    return middle_sup

    def get_out_of_obstacle(self):
        """
        Get out of the obstacle
        """
        escape_point = None
        r = 1
        while type(escape_point) == type(None):
            # Solve x+y = r
            for x in range(-r, r+1):

                y = r - abs(x)
                (x_conv, y_conv) = (x*0.01, y*0.01)
                if not(self.is_obstacle(self.pos[0] + x_conv, self.pos[1] + y_conv)):
                    escape_point = np.array([self.pos[0] + x_conv, self.pos[1] + y_conv])
                if not(self.is_obstacle(self.pos[0] + x_conv, self.pos[1] - y_conv)):
                    escape_point = np.array([self.pos[0] + x_conv, self.pos[1] - y_conv])
            r += 1
        self.path = [self.pos, escape_point]

    def master_path(self, start, end):
        """
        Calculate the path to go from start to end
        """

        if self.is_obstacle(start[0], start[1]) or self.is_obstacle(end[0], end[1]):
            rospy.logwarn("Départ ou arrivée dans un obstacle")
            if self.is_obstacle(start[0], start[1]):
                rospy.logwarn("Départ dans un obstacle, on le quitte")
                self.get_out_of_obstacle()
                return None

        path = [start, end]
        self.position_goal = end

        path_portion = 0

        iter = 0
        max_iter = self.max_iter

        while path_portion<(len(path)-1) and iter < max_iter:
            iter += 1
            middle = self.find_middle_obstacles(path, path_portion)
            if middle:
                detour = self.find_normal_non_obstacle(middle[0], middle[1])
                if type(detour) != type(None):
                    path.insert(path_portion+1, detour)
                else:
                    rospy.logwarn("Pas de trajectoire")
                    return None
            else:
                path_portion += 1

        if iter == max_iter:
            rospy.logwarn("Trop d'itération :" + str(max_iter) + " itérations")
            return None

        self.path = path

    def correct_path(self):
        """
        Correct the path if an obstacle is on it
        """

        # Vérifie si un obstacle est sur le chemin
        pos = self.position
        for i in range(len(self.path)-1):
            vect = np.array(self.path[i+1]) - np.array(self.path[i])/np.linalg.norm(np.array(self.path[i+1]) - np.array(self.path[i]))
            if np.linalg.norm(np.array(pos) - np.array(self.path[i])) < 0.1:
                continue
            else:
                pos += vect * 0.01

            if self.is_obstacle(pos[0], pos[1]):
                rospy.logwarn("Obstacle sur le chemin")
                self.master_path(self.position, self.position_goal)
                return None
        

    # Callbacks
    # ---------------------------------------------------------------------------------------------

    def position_goal_callback(self, msg):
        """
        Callback pour récupérer la position cible
        """
        rospy.logdebug("Position goal : " + str(msg))
        self.position_goal = [msg.x, msg.y]
        self.master_path(self.position, self.position_goal)
        
    def publish_pic_msg(self, next_goal):
        """
        Publie un message ordonnant au robot de suivre le chemin "path"

        Paramètres
        ----------
            - path : liste\n
                Liste ordonnée des noeuds constituant le chemin
        """
        msg = Pic_Action()
        msg.action_destination = 'motor'
        msg.action_msg = 'MOVE'
        msg.action_msg += ' ' + str(next_goal[0]) + ' ' + str(next_goal[1])
        self.action_orders_pub.publish(msg)

        if self.simu_mode:
            action_client_goal = Virtual_Robot_ActionGoal()
            action_client_goal.command = msg.action_msg
            action_client.send_goal(action_client_goal)
        
    def robot_data_callback(self, msg):
        self.robot_data = msg
        self.position = np.array([msg.position.x, msg.position.y])
        self.orientation = msg.position.z        

        if self.path:
            if type(self.next_goal) == type(None):
                vect = np.array(self.path[1]) - np.array(self.position)/np.linalg.norm(np.array(self.path[1]) - np.array(self.position))
                self.next_goal = np.array(self.position) + vect * self.distance_interpoint
            else:
                if np.linalg.norm(np.array(self.position) - np.array(self.next_goal)) < self.distance_interpoint:
                    if np.linalg.norm(np.array(self.position) - np.array(self.path[0])) < self.distance_interpoint:
                        self.path.pop(0)
                        self.next_goal = self.path[0]
                    else:
                        vect = np.array(self.path[1]) - np.array(self.position)/np.linalg.norm(np.array(self.path[1]) - np.array(self.position))
                        self.next_goal = self.position + vect * self.distance_interpoint
                    self.publish_pic_msg(self.next_goal)

    def emergency_stop(self):
        """
        Stoppe le robot
        """
        self.path=[self.position]
        rospy.logwarn("STOP")

    def obstacles_callback(self, msg):
        """
        Callback pour récupérer les obstacles
        """
        
        # Emergency stop
        if self.emergency_stop_distance > 0:
            for obstacle in msg.other_robot_lidar:
                arr = self.chgt_base_plateau_to_robot(obstacle)
                if abs(arr[1]) < self.emergency_stop_distance and abs(arr[1]) < self.max_radius/2:
                    if self.robot_data.linear_velocity > 0: # Le robot va en avant
                        if arr[0] > 0:
                            self.emergency_stop()
                    else:
                        if arr[0]<0:
                            self.emergency_stop()


        ## On récupère les obstacles
        Liste_obstacle = [(obstacle.position.x, obstacle.position.y, self.max_radius) for obstacle in msg.other_robot_lidar]

        ## On ne garde que les obstacles qui sont sur le plateau ou à moins de 50cm du plateau
        Obstacles_coherents = []

        x_plateau = self.board[0]
        y_plateau = self.board[1]

        for obstacle in Liste_obstacle:                            # Tout objet à plus de 50 cm du plateau est ignoré
            if (-50<obstacle[0] and obstacle[0]<x_plateau+50) and (-50<obstacle[1] and obstacle[1]<y_plateau+50) and (obstacle[2]<50):
                Obstacles_coherents.append(obstacle)

        ## On crée une aire de jeu plus grande que le plateau pour traiter les obstacles hors du plateau

        x_plateau_max = x_plateau + 200                    # On rajoute 100cm de marge sur chaque côté
        y_plateau_max = y_plateau + 200

        Cadrillage_max = np.zeros((x_plateau_max, y_plateau_max))

        ## On crée une matrice associée à chaque obstacle
        
        for obstacle in Liste_obstacle:
            x_obstacle = obstacle[0]+100
            y_obstacle = obstacle[1]+100
            rayon_obstacle = obstacle[2]

            # Placer des 1 dans le cercle de rayon rayon_obstacle autour de coordonnees_obstacle
            carre_rayon = rayon_obstacle**2
            M_obstacle = ((np.arange(2*rayon_obstacle)-rayon_obstacle)**2+((np.arange(2*rayon_obstacle)-rayon_obstacle)**2).reshape(2*rayon_obstacle,1)<=carre_rayon).astype(int)

            # On place la matrice dans la matrice Cadrillage_max aux bonnes coordonnées
            Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle]= np.logical_or(Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle],M_obstacle)

        ## On crée une matrice associée au plateau
        Cadrillage_rempli = Cadrillage_max[100:x_plateau_max-100,100:y_plateau+100]

        ## On met à jour les informations sur le plateau
        self.map_obstacles = np.logical_or(self.static_obstacles, Cadrillage_rempli)
                 
        self.next_point = msg
    
        pass
        
    def chgt_base_plateau_to_robot(self, point):
        cos_angle = np.cos(self.orientation)
        sin_angle = np.sin(self.orientation)
        point_transforme_to_robot = np.array([0,0])
        point_transforme_to_robot[0] = (point.x - self.position[0]) * cos_angle + (point.y - self.position[1]) * sin_angle
        point_transforme_to_robot[1] = (self.orientation[0] - point.x) * sin_angle + (point.y - self.orientation[1]) * cos_angle
        return point_transforme_to_robot

if __name__ == '__main__':
    rospy.init_node('nav_node', anonymous=False)

    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_1/Pos_goal')
    obstacles_topic = rospy.get_param('~obstacles_topic', '/obstacles')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_1/action')
    debug_mode = rospy.get_param('~debug_mode', False)
    max_iter = rospy.get_param('~max_iter', 15)
    distance_interpoint = rospy.get_param('~distance_interpoint', 0.03)
    margin = rospy.get_param('~margin', 0.1) #m
    emergency_stop_distance = rospy.get_param('~emergency_stop_distance', 0.2) #m
    robot_data_topic = rospy.get_param('~robot_data_topic', '/robot_1/Odom')
    simu_mode = rospy.get_param('~simu_mode', True)

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=1)

    if debug_mode:
        action_client = actionlib.SimpleActionClient('pic_action', Virtual_Robot_ActionAction)

    # Création de la classe NavigationNode
    Nav_node = NavNode(action_orders_pub=action_orders_pub, 
                       distance_interpoint=distance_interpoint, 
                       margin=margin, 
                       max_iter=max_iter,
                       emergency_stop_distance=emergency_stop_distance,
                       simu_mode=simu_mode)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(robot_data_topic, RobotData, Nav_node.robot_data_callback)
    rospy.Subscriber(obstacles_topic, Obstacles, Nav_node.obstacles_callback)
    rospy.Subscriber('/robot_x/motion_done', Bool, Nav_node.motion_done_callback)

    # Vérification de la présence d'obstacle sur le chemin du robot
    while not rospy.is_shutdown():

        rospy.sleep(0.05)