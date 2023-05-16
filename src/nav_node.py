import rospy, rospkg
from geometry_msgs.msg import Point
from obstacle_detector.msg import Obstacles
from cdf_msgs.msg import Pic_Action, RobotData, MergedDataBis, Trajectoire
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray, Marker
from tool_lidar.objet import ChooseColor


import tool_lidar.variable_globale as tool_glob
import tool_lidar.publisher as tool_pub
import numpy as np

class NavNode():
    def __init__(self,
                 action_orders_pub,
                 action_result_pub,
                 margin=0.03, 
                 pos = np.array([0.5,1]), 
                 max_radius = 0.45, 
                 max_iter = 15, 
                 distance_interpoint = 0.03, 
                 name_robot = "Han7",
                 debug = False):
        self.pub_marker = rospy.Publisher(rospy.get_param("~pub_marker_next_pos", "/robot_1/marker_next_pos"), MarkerArray, queue_size = 10)
        self.path = []
        self.margin = margin
        self.position_goal = None
        self.max_iter = max_iter
        self.debug = debug
        self.standby = False

        rospack = rospkg.RosPack()
        rospack.list()

        self.static_obstacles = np.load("{}/src/map/base.npy".format(rospack.get_path('nav_node')))
        self.dict_map ={}
        self.load_all_maps()

        self.map_obstacles = self.static_obstacles.copy()

        self.max_radius = max_radius
        self.shape_board = self.map_obstacles.shape

        if self.shape_board[0] == 200:
            self.map_obstacles = np.transpose(self.map_obstacles)
            self.static_obstacles = np.transpose(self.static_obstacles)
            self.shape_board = self.map_obstacles.shape
            rospy.logwarn("Transposition de la map")
        
        self.position = pos

        self.distance_interpoint = distance_interpoint
        self.next_goal = None

        self.action_orders_pub = action_orders_pub
        self.action_result_pub = action_result_pub

        self.name_robot = name_robot

        # Contain the list of the obstacles (valeurs initiales aberrantes)
        self.obstacles = [np.array([1000,1000]) for _ in range(3)]
        self.is_in_obstacle = False

        if self.debug:
            # Launch callback every 100ms
            self.react_pub = rospy.Publisher('/robot_2/positions_topic', MergedDataBis, queue_size=1)
            self.react_pub_pos = rospy.Publisher('/robot_2/Odom', RobotData, queue_size=1)
            rospy.Timer(rospy.Duration(0.1), self.debug_callback)
            self.t = 0  

        rospy.loginfo("Nav node initialized with shape : " + str(self.shape_board))

    def load_all_maps(self):
        rospack = rospkg.RosPack()
        rospack.list()
        l = {}
        j1 = np.load("{}/src/map/j1.npy".format(rospack.get_path('nav_node')))
        j2 = np.load("{}/src/map/j2.npy".format(rospack.get_path('nav_node')))
        j3 = np.load("{}/src/map/j3.npy".format(rospack.get_path('nav_node')))
        j4 = np.load("{}/src/map/j4.npy".format(rospack.get_path('nav_node')))
        r1 = np.load("{}/src/map/r1.npy".format(rospack.get_path('nav_node')))
        r2 = np.load("{}/src/map/r2.npy".format(rospack.get_path('nav_node')))
        r3 = np.load("{}/src/map/r3.npy".format(rospack.get_path('nav_node')))
        r4 = np.load("{}/src/map/r4.npy".format(rospack.get_path('nav_node')))
        p1 = np.load("{}/src/map/p1.npy".format(rospack.get_path('nav_node')))
        p2 = np.load("{}/src/map/p2.npy".format(rospack.get_path('nav_node')))
        p3 = np.load("{}/src/map/p3.npy".format(rospack.get_path('nav_node')))
        p4 = np.load("{}/src/map/p4.npy".format(rospack.get_path('nav_node')))
        p5 = np.load("{}/src/map/p5.npy".format(rospack.get_path('nav_node')))
        p6 = np.load("{}/src/map/p6.npy".format(rospack.get_path('nav_node')))
        p7 = np.load("{}/src/map/p7.npy".format(rospack.get_path('nav_node')))
        p8 = np.load("{}/src/map/p8.npy".format(rospack.get_path('nav_node')))
        p9 = np.load("{}/src/map/p9.npy".format(rospack.get_path('nav_node')))
        p10 = np.load("{}/src/map/p10.npy".format(rospack.get_path('nav_node')))

        l["j1"] = j1
        l["j2"] = j2
        l["j3"] = j3
        l["j4"] = j4
        l["r1"] = r1
        l["r2"] = r2
        l["r3"] = r3
        l["r4"] = r4
        l["p1"] = p1
        l["p2"] = p2
        l["p3"] = p3
        l["p4"] = p4
        l["p5"] = p5
        l["p6"] = p6
        l["p7"] = p7
        l["p8"] = p8
        l["p9"] = p9
        l["p10"] = p10

        self.dict_map = l

    def find_middle_obstacles(self, path, path_portion : int):
        """
        Find the beginning and the end of the obstacle in the path portion

        Parameters
        ----------
        path_portion : int Portion de chemin

        Returns
        -------
        (start_obstacle, end_obstacle) : tuple
        False if no obstacle

        """
        start = np.array(path[path_portion])
        end = np.array(path[path_portion + 1])

        discretisation = np.linspace(start, end, int(np.linalg.norm(end-start)/0.01))

        index_in_disc = 0
        (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])

        # Find the beginning of the obstacle
        while not(self.is_obstacle(x,y)) and index_in_disc < len(discretisation) - 1:
            index_in_disc += 1
            (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])
        
        if index_in_disc == len(discretisation) - 1:
            return False
        elif index_in_disc == 0:
            #l'obstacle se trouve à l'index 0, nous sommes dans un obstacle
            start_obstacle = discretisation[0]
        else:
            #On a trouvé un obstacle
            start_obstacle = discretisation[index_in_disc-1]
        
        start_obstacle = discretisation[index_in_disc]

        # Find the end of the obstacle
        while self.is_obstacle(x,y) and index_in_disc < len(discretisation) - 1:
            index_in_disc += 1
            (x, y) = (discretisation[index_in_disc, 0], discretisation[index_in_disc, 1])
        
        rospy.loginfo("Obstacle trouvé : " + str(start_obstacle) + " " + str(discretisation[index_in_disc]))
        if index_in_disc == len(discretisation) - 1:
            #rospy.logwarn("Obstacle non fermé")
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
        rospy.loginfo("Recherche d'un détour à partir de l'obstacle {} {}".format(start, end))

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
                    rospy.logwarn("ATTENTION : Chemin étroit trouvé en ")
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
        self.is_in_obstacle = True
        escape_point = None
        r = 1
        while type(escape_point) == type(None):
            # Solve x+y = r
            for x in range(-r, r+1):

                y = r - abs(x)
                (x_conv, y_conv) = (x*0.01, y*0.01)
                if not(self.is_obstacle(self.position[0] + x_conv, self.position[1] + y_conv)):
                    escape_point = np.array([self.position[0] + x_conv, self.position[1] + y_conv])
                if not(self.is_obstacle(self.position[0] + x_conv, self.position[1] - y_conv)):
                    escape_point = np.array([self.position[0] + x_conv, self.position[1] - y_conv])
            r += 1
        rospy.loginfo("Sortie d'obstacle trouvé en " + str(escape_point))
        rospy.loginfo("Obstacles : " + str(self.obstacles))
        self.path = [escape_point]
        self.next_goal = escape_point
        #rospy.loginfo("Point de sortie trouvé : " + str(escape_point))
        self.publish_pic_msg(escape_point)

    def master_path(self, start, end, apply = True):
        """
        Calculate the path to go from start to end
        """

        """ if self.is_obstacle(start[0], start[1]) or self.is_obstacle(end[0], end[1]):
            # rospy.logwarn("Départ ou arrivée dans un obstacle")
            if self.is_obstacle(start[0], start[1]):
                # rospy.logwarn("Départ dans un obstacle, on le quitte")
                #TODO : Fiabiliser la sortie d'obstacle
                self.get_out_of_obstacle()
                return None
            else:
                rospy.logwarn("Arrivée dans un obstacle ! Pas de chemin : " + str(end))
                rospy.logwarn("Obstacles : "+str(self.obstacles))
        """
        path = [start, end]
        self.position_goal = end

        path_portion = 0

        iter = 0
        max_iter = self.max_iter

        while path_portion<(len(path)-1) and iter < max_iter:
            iter += 1
            middle = self.find_middle_obstacles(path, path_portion)
            if type(middle) == type(None):
                self.path = None
                rospy.logwarn("Pas de trajectoire 1")
                return None
            elif middle:
                detour = self.find_normal_non_obstacle(middle[0], middle[1])
                rospy.loginfo("Detour trouvé : " + str(detour))
                if type(detour) != type(None):
                    path.insert(path_portion+1, detour)
                else:
                    self.path = None
                    rospy.logwarn("Pas de trajectoire 2")
                    return None
            else:
                path_portion += 1

        if iter == max_iter:
            rospy.logwarn("Trop d'itération :" + str(max_iter) + " itérations")
            return None
        else:
            #
            #rospy.loginfo("Found path" + str(path))
            pass


        path = path[1:]


        if self.verify_path():
            rospy.logerr("OBSTACLE SUR LE CHEMIN")

        if apply:
            self.path = path
            rospy.loginfo("Found path : " + str(self.path))
            return True
        else:
            return path
        
    def get_next_pos(self):
        """
        Get the next position to go and store it in self.next_goal
        """
        if len(self.path)!=0:
            self.next_goal = self.path[0]
        else:
            self.next_goal = self.position
        rospy.loginfo("New next goal : "+str(self.next_goal))
        return self.next_goal
        
    def obstacle_variation(self, liste_obstacle):
        """
        Check if the obstacle list has changed and return a boolean
        """
        if len(self.obstacles)!=len(liste_obstacle):
            return True
        for prev_obs in self.obstacles:
            b=False
            for obs in liste_obstacle:
                #Trigger distance = 0.03
                b = b or np.linalg.norm(np.array(obs) - np.array(prev_obs)) < 10
            if b==False: # Aucun des nouveaux obstacles n'est proche de cet ancien
                return True
        return False
            
    def verify_path(self):
        """
        Verify if there is an obstacle on the path
        """
        if self.path == []:
            return False
        points = np.linspace(self.path[0], self.position, int(np.linalg.norm(np.array(self.path[0]) - np.array(self.position))/0.01))
        for point in points:
                if self.is_obstacle(point[0], point[1]):
                    rospy.loginfo("Obstacle :" + str(point))
                    return True
        for i in range(len(self.path)-1):
            # Find each cm of the path
            points = np.linspace(self.path[i], self.path[i+1], int(np.linalg.norm(np.array(self.path[i+1]) - np.array(self.path[i]))/0.01))
            for point in points:
                if self.is_obstacle(point[0], point[1]):
                    rospy.loginfo("Obstacle :" + str(point))
                    return True
        #rospy.loginfo("No obstacle on the path")
        #rospy.loginfo("Obstacles :" + str(self.obstacles))
        return False
    
    def add_obstacles(self, liste_obstacle):
        self.obstacles = liste_obstacle

        ## On ne garde que les obstacles qui sont sur le plateau ou à moins de 50cm du plateau
        Obstacles_coherents = []

        x_plateau = self.shape_board[0]
        y_plateau = self.shape_board[1]

        for obstacle in liste_obstacle:                            # Tout objet à plus de 50 cm du plateau est ignoré
            if (-50<obstacle[0] and obstacle[0]<x_plateau+50) and (-50<obstacle[1] and obstacle[1]<y_plateau+50):
                Obstacles_coherents.append(obstacle)

        ## On crée une aire de jeu plus grande que le plateau pour traiter les obstacles hors du plateau

        x_plateau_max = x_plateau + 200                    # On rajoute 100cm de marge sur chaque côté
        y_plateau_max = y_plateau + 200

        Cadrillage_max = np.zeros((x_plateau_max, y_plateau_max))

        ## On crée une matrice associée à chaque obstacle

        for obstacle in Obstacles_coherents: 
            x_obstacle = int(obstacle[0])+100
            y_obstacle = int(obstacle[1])+100
            rayon_obstacle = int(100*self.max_radius)

            # Placer des 1 dans le cercle de rayon rayon_obstacle autour de coordonnees_obstacle
            carre_rayon = rayon_obstacle**2
            M_obstacle = ((np.arange(2*rayon_obstacle)-rayon_obstacle)**2+((np.arange(2*rayon_obstacle)-rayon_obstacle)**2).reshape(2*rayon_obstacle,1)<=carre_rayon).astype(int)

            # On place la matrice dans la matrice Cadrillage_max aux bonnes coordonnées
            Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle]= np.logical_or(Cadrillage_max[x_obstacle-rayon_obstacle:x_obstacle+rayon_obstacle,y_obstacle-rayon_obstacle:y_obstacle+rayon_obstacle],M_obstacle)

        ## On crée une matrice associée au plateau
        Cadrillage_rempli = Cadrillage_max[100:x_plateau_max-100,100:y_plateau+100]

        ## On met à jour les informations sur le plateau
        self.map_obstacles = np.logical_or(self.static_obstacles, Cadrillage_rempli)

    def assign_position(self, msg):
        """
        Récupération des positions des robots
        """

        if self.name_robot == "Han7":
            liste_obstacle = []
            if not(msg.robot_2[-1].position.x == 0 and msg.robot_2[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.robot_2[-1].position.x, msg.robot_2[-1].position.y]))
            if not(msg.ennemi_1[-1].position.x == 0 and msg.ennemi_1[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_1[-1].position.x, msg.ennemi_1[-1].position.y]))
            if not(msg.ennemi_2[-1].position.x == 0 and msg.ennemi_2[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_2[-1].position.x, msg.ennemi_2[-1].position.y]))
            if not(msg.ennemi_3[-1].position.x == 0 and msg.ennemi_3[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_3[-1].position.x, msg.ennemi_3[-1].position.y]))
        elif self.name_robot == "Gret7" :
            liste_obstacle = []
            if not(msg.robot_1[-1].position.x == 0 and msg.robot_1[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.robot_1[-1].position.x, msg.robot_1[-1].position.y]))
            if not(msg.ennemi_1[-1].position.x == 0 and msg.ennemi_1[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_1[-1].position.x, msg.ennemi_1[-1].position.y]))
            if not(msg.ennemi_2[-1].position.x == 0 and msg.ennemi_2[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_2[-1].position.x, msg.ennemi_2[-1].position.y]))
            if not(msg.ennemi_3[-1].position.x == 0 and msg.ennemi_3[-1].position.y == 0):
                liste_obstacle.append(np.array([msg.ennemi_3[-1].position.x, msg.ennemi_3[-1].position.y]))
        else :
            rospy.logerr("Nom de robot non reconnu")    
        return liste_obstacle

    # Callbacks
    # ---------------------------------------------------------------------------------------------
    def odometry_callback(self,msg):
        """
        Get position from odometry
        """
        self.position = np.array([msg.position.x, msg.position.y])

    def position_goal_callback(self, msg):
        """
        Callback pour récupérer la position cible
        """

        rospy.loginfo("Shape : " + str(self.map_obstacles.shape))

        dist = np.sqrt((msg.x - self.position[0])**2 + (msg.y - self.position[1])**2)
        if dist > 0.01: # Si la position cible est trop loin
            self.position_goal = [msg.x, msg.y]
            rospy.loginfo("Position goal : " + str(msg))
            color = ChooseColor(1, 0, 0)
            marker_array = MarkerArray()
            marker_pos_other = tool_pub.create_marker(800, 3, 0, 0.1, 0.1, msg.x, msg.y, color, scale_z=0.15, frame_id="fuzed") 
            marker_array.markers.append(marker_pos_other)
            self.pub_marker.publish(marker_array)
            res = self.master_path(self.position, self.position_goal)
            if res == None:
                self.publish_pic_msg(self.position)
                self.action_result_pub.publish(False)
            else:
                # On réduit le rayon de courbure pour tourner sur soi-même
                msg_rayoncourbure = Pic_Action()

                msg_rayoncourbure.action_destination = 'motor'
                msg_rayoncourbure.action_msg = 'setrayoncourbure'
                msg_rayoncourbure.action_msg += ' ' + str(0.001)

                self.action_orders_pub.publish(msg_rayoncourbure)

                self.get_next_pos()
                self.publish_pic_msg(self.next_goal)
        else : # La cible est proche
            rospy.loginfo("Cible atteinte")
            self.position_goal = self.position
            self.path = [np.array(self.position)]

    def publish_pic_msg(self, next_goal, more_param = 0):
        """
        Publie un message ordonnant au robot de suivre le chemin "path"

        Paramètres
        ----------
            - path : liste\n
                Liste ordonnée des noeuds constituant le chemin
        """
        

        msg = Pic_Action()
        msg.action_destination = 'motor'
        msg.action_msg = 'moveavant'
        msg.action_msg += ' ' + str(next_goal[0]) + ' ' + str(next_goal[1]) + ' ' + str(more_param)

        self.action_orders_pub.publish(msg)
    
    def emergency_stop_callback(self, msg):
        """
        Stoppe le robot
        """
        if msg.data == True:
            self.standby = True
        elif msg.data == False:
            self.standby = False
            self.publish_pic_msg(self.next_goal)
            rospy.loginfo("Reprise du chemin")

    def position_callback(self, msg):
        """
        Callback pour récupérer la position des robots
        """
        try:
            for i, elem in enumerate(self.path):
                color = ChooseColor(1, 0, 0)
                marker_array = MarkerArray()
                marker_pos_other = tool_pub.create_marker(2000 + i, 3, 0, 0.1, 0.1, elem[0], elem[1], color, scale_z=0.15, frame_id="/fuzed") 
                marker_array.markers.append(marker_pos_other)
                self.pub_marker.publish(marker_array)
        except TypeError:
            pass

        if self.standby:
            return None
 
        liste_obstacle = self.assign_position(msg)

        self.obstacles_processing(liste_obstacle)

    
        if type(self.next_goal) != type(None):    
            if np.linalg.norm(self.position - self.next_goal) < self.distance_interpoint:
                if len(self.path) >= 2:
                    self.path.pop(0)
                    self.next_goal = self.get_next_pos()
                    if len(self.path) == 2:
                        self.publish_pic_msg(self.next_goal)
                    else:
                        self.publish_pic_msg(self.next_goal)
                elif len(self.path) == 1:
                    self.path = []
                    self.next_goal = None
                    self.position_goal = None
                    rospy.loginfo("Arrivé à destination : " + str(self.position))
                    self.action_result_pub.publish(True)
                else:
                    rospy.logwarn("Unexpected Warning")
            else:
                #rospy.loginfo("Dist : " +str(np.linalg.norm(self.position - self.next_goal)))
                pass

    def obstacles_processing(self, liste_obstacle):
        """
        Fonction pour récupérer les obstacles
        """
        
        # Convert to cm
        liste_obstacle = np.array([obstacle*100 for obstacle in liste_obstacle])

        if self.obstacle_variation(liste_obstacle):
            
            #rospy.loginfo("New_obstacles : " + str(liste_obstacle))

            self.obstacles = liste_obstacle

            #On ajoute les obstacles à la carte
            self.add_obstacles(liste_obstacle)
    
    def activation_callback(self, msg):
        self.position_goal = None
        self.next_goal = None

    def debug_callback(self, data):
        self.t += 1
        msg = MergedDataBis()
        if type(self.next_goal) == type(None):
            following_point = self.position
        elif np.linalg.norm(self.next_goal - self.position)<0.005:
            following_point = self.position
        else:
            following_point = self.position + (self.next_goal - self.position)/np.linalg.norm(self.next_goal - self.position)*0.005
        msg.robot_1 = [Trajectoire()]
        msg.robot_1[0].position = Point(following_point[0], following_point[1], 0)
        msg.robot_2 = [Trajectoire()]
        msg.robot_2[0].position = Point(0, 0, 0)
        msg.ennemi_1 = [Trajectoire()]
        msg.ennemi_1[0].position = Point(0, 0, 0)
        msg.ennemi_2 = [Trajectoire()]
        msg.ennemi_2[0].position = Point(0, 0, 0)
        msg.ennemi_3 = [Trajectoire()]
        msg.ennemi_3[0].position = Point(1.2, 1, 0)

        msg2 = RobotData()
        msg2.position = Point(following_point[0], following_point[1], 0)


        try:
            if np.linalg.norm(self.prec_position - self.position)>0.01:
                rospy.loginfo("[DEBUG] Publishing : " + str(following_point) + " with ennemies : " + str(msg.ennemi_3[0].position))
                self.prec_position = self.position
        except AttributeError:
            self.prec_position = self.position
            rospy.loginfo("[DEBUG] Publishing : " + str(following_point))
        self.react_pub.publish(msg)
        self.react_pub_pos.publish(msg2)

    def load_map_callback(self, s):
        """
        Callback pour récupérer la map
        """
        l = len(s.data)
        map = np.array(self.map_obstacles.shape)
        for i in range(l//2):
            add_map = self.dict_map[s.data[2*i:2*i+2]]
            map = np.logical_or(map, add_map)
        self.map_obstacles = map

        

if __name__ == '__main__':

    rospy.init_node('nav_node', anonymous=False)
    pub_marker = rospy.Publisher(rospy.get_param("~pub_marker_other", "/robot_1/marker_obs"), MarkerArray, queue_size = 10)
    position_goal_topic = rospy.get_param('~position_goal_topic', '/robot_1/Pos_goal')
    action_orders_topic = rospy.get_param('~action_orders_topic', '/robot_1/action')
    nav_node_result_topic = rospy.get_param('~nav_node_result_topic', '/robot_1/nav_node_result')
    max_iter = rospy.get_param('~max_iter', 15)
    distance_interpoint = rospy.get_param('~distance_interpoint', 0.03)
    margin = rospy.get_param('~margin', 0.1) #m
    emergency_topic = rospy.get_param('~emergency_topic', '/robot_1/obstacle_detected')
    positions_topic = rospy.get_param('~positions_topic', '/robot_1/positions_topic') 
    name_robot = rospy.get_param('~name_robot', 'Han7')
    debug_mode = rospy.get_param('~debug_mode', False)
    activate_topic = rospy.get_param('~activate_topic', '/robot_1/activation_nav_node')
    odometry_topic = rospy.get_param('~odometry_topic', '/robot_1/Odom')
    obstacle_map_topic = rospy.get_param('~obstacle_map_topic', '/robot_1/nav_node_obstacle_map')

    # Déclaration des Publishers
    action_orders_pub = rospy.Publisher(action_orders_topic, Pic_Action, queue_size=1)
    result_pub = rospy.Publisher(nav_node_result_topic, Bool, queue_size=1)

    # Création de la classe NavigationNode
    Nav_node = NavNode(action_orders_pub=action_orders_pub, 
                       action_result_pub=result_pub,
                       distance_interpoint=distance_interpoint, 
                       margin=margin, 
                       max_iter=max_iter,
                       name_robot=name_robot,
                       debug = debug_mode)

    # Déclaration des Subscribers
    rospy.Subscriber(position_goal_topic, Point, Nav_node.position_goal_callback)
    rospy.Subscriber(positions_topic, MergedDataBis, Nav_node.position_callback)
    rospy.Subscriber(activate_topic, Bool, Nav_node.activation_callback)
    rospy.Subscriber(odometry_topic, RobotData, Nav_node.odometry_callback)
    rospy.Subscriber(emergency_topic, Bool, Nav_node.emergency_stop_callback)
    rospy.Subscriber(obstacle_map_topic, String, Nav_node.load_map_callback)

    while not rospy.is_shutdown():

        rospy.sleep(0.05)