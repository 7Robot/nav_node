import rospy
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import RobotData, Pic_Action
from virtual_robot.msg import Virtual_Robot_ActionAction, Virtual_Robot_ActionGoal
from std_msgs.msg import Bool
import numpy as np
import actionlib

class NavNode():
    def __init__(self, margin=0.03, pos = np.array([0,0]), max_radius = 0.35):
        self.path = []
        self.margin = margin
        self.position_goal = None

        self.map_static_obstacles = np.load("cvmap2.npy")
        self.map_obstacles = self.map_static_obstacles.copy()
        self.max_radius = max_radius
        
        self.robot_data = RobotData()

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
            print("ERREUR : obstacle non fermé")
            return None
        else:
            end_obstacle = discretisation[index_in_disc]

            return (start_obstacle, end_obstacle)
        
    def is_obstacle(self, x, y):    # modifier pour que collision soit un encadrement et pas une valeur exacte ?
        """
        Vérifie si le point (x, y) touche un obstacle
        """
        (xf,yf) = (int(x*100), int(y*100))
        #print("x : ", xf, "y : ", yf)
        if xf < 0 or yf < 0 or xf >= 200 or yf >= 300:
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
        while (self.is_obstacle(middle_sup[0],middle_sup[1]) and self.is_obstacle(middle_inf[0], middle_inf[1])) and dist < 3.5:
            middle_sup += vec_norm  
            middle_inf -= vec_norm    
            dist += 0.01

        # Return the nearest point
        if dist > 3.5:
            print("ERREUR : pas de point millieu trouvé")
            return None      # No point found
        else:
            if self.is_obstacle(middle_sup[0], middle_sup[1]):
                middle_inf_decal = middle_inf - self.margin * vec_norm * 100
                if not(self.is_obstacle(middle_inf_decal[0], middle_inf_decal[1])):
                    return middle_inf - self.margin * vec_norm * 100
                else:
                    print("ATTENTION : Chemin étroit")
                    return middle_inf
            else:
                middle_sup_decal = middle_sup + self.margin * vec_norm * 100
                if not(self.is_obstacle(middle_sup_decal[0], middle_sup_decal[1])):
                    return middle_sup + self.margin * vec_norm * 100
                else:
                    print("ATTENTION : Chemin étroit")
                    return middle_sup

    def master_path(self, start, end):
        """
        Calculate the path to go from start to end
        """

        path = [start, end]
        self.position_goal = end

        path_portion = 0

        iter = 0
        max_iter = 15

        while path_portion<(len(path)-1) and iter < max_iter:
            iter += 1
            #print("path : ", path)
            #print("path_portion : ", path_portion)
            middle = self.find_middle_obstacles(path, path_portion)
            if middle:
                detour = self.find_normal_non_obstacle(middle[0], middle[1])
                print("detour : ", detour)
                if type(detour) != type(None):
                    path.insert(path_portion+1, detour)
                else:
                    print("ERREUR : pas de trajectoire")
                    return None
            else:
                path_portion += 1

        if iter == max_iter:
            print("ERREUR : trop d'itérations")
            return None

        self.path = path


    # Callbacks

    def position_goal_callback(self, msg):
        """
        Callback pour récupérer la position cible
        """
        print("Position goal : " + str(msg))
        self.position_goal = [msg.x, msg.y]
        
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

        if debug_mode:
            action_client_goal = Virtual_Robot_ActionGoal()
            action_client_goal.command = msg.action_msg
            action_client.send_goal(action_client_goal)
        
    def robot_data_callback(self, msg):
        self.robot_data = msg

    def obstacles_callback(self, msg):
        """
        Callback pour récupérer les obstacles
        """
        
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

                  
    def next_point_callback(self, msg):
        self.next_point = msg
    
    def motion_done_callback(self, msg):
        pass
        
if __name__ == '__main__':
    rospy.init_node('navigation_node', anonymous=False)

    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_x/Pos_goal')
    obstacles_topic = rospy.get_param('~obstacles_topic', '/obstacles')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_x/action')
    next_point_topic = rospy.get_param('~next_point_topic', '/robot_x/asserv_next_point')
    graph_file = rospy.get_param('~graph_file', 'default')
    cv_obstacle_file = rospy.get_param('~cv_obstacle_file', 'default')
    debug_mode = rospy.get_param('~debug_mode', False)
    distance_interpoint = rospy.get_param('~distance_interpoint', '15')
    margin = rospy.get_param('~margin', 0.1) #m
    emergency_stop_distance = rospy.get_param('~emergency_stop_distance', 0.2) #m
    robot_data_topic = rospy.get_param('~robot_data_topic', '/robot_x/Odom')
    robot_x_dimension = rospy.get_param('~robot_x_dimension', 0.5) #m
    robot_y_dimension = rospy.get_param('~robot_y_dimension', 0.2) #m

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=1)

    if debug_mode:
        action_client = actionlib.SimpleActionClient('pic_action', Virtual_Robot_ActionAction)

    # Création de la classe NavigationNode
    Nav_node = NavigationNode(distance_interpoint, margin)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(robot_data_topic, RobotData, Nav_node.robot_data_callback)
    rospy.Subscriber(obstacles_topic, Obstacles, Nav_node.obstacles_callback)
    rospy.Subscriber(next_point_topic, Point, Nav_node.next_point_callback)
    rospy.Subscriber('/robot_x/motion_done', Bool, Nav_node.motion_done_callback)

    # Vérification de la présence d'obstacle sur le chemin du robot
    while not rospy.is_shutdown():

        rospy.sleep(0.05)