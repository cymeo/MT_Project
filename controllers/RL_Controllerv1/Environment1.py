import From_Webot1 as FW
from gymnasium import Env
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation as R


## simple environement, actionspace is plus or minas motorjoint speeds 


class WeBot_environment(Env):
    """Custom Environment that follows gym interface."""

    def __init__(self):
        ######### rewards for -distance to goal,-rotational_distance, success, -max_steps, crash ############
        self.weights = np.array([0.1,0.0,10,50]) 
        self.max_step = 1000
        super().__init__()
        # Define action and observation space
        #action = Motorangle steps  
        self.action_space = spaces.Box(low= -np.pi, high = np.pi, shape = (6,))
        #observation = Endeffector pose, motor angles, Goal Pose, 
        self.reset_pose = np.array([0,-np.pi/2, np.pi/2, -np.pi/2,-np.pi/2,0])
        self.observation_space = spaces.Dict(
            {
                "goal":spaces.Box(
                    low = np.array([-1,-1,-1]),
                    high =np.array([1,1,1]), 
                    dtype = float    
                ),
            
                "p_ee": spaces.Box(
                    low = np.array([-1.2,-1.2,-1.2]),
                    high =np.array([1.2,1.2,1.2]), 
                    dtype = float
                    ), 
                       
                "q_ee": spaces.Box(
                    low = np.array([-1,-1,-1,-1]),
                    high =np.array([1,1,1,1]), 
                    dtype = float
                    ),

                "theta": spaces.Box(
                        low = np.array([-2*np.pi, -2*np.pi, -2*np.pi,-2*np.pi, -2*np.pi, -2*np.pi]),
                        high =np.array([2*np.pi, 2*np.pi, 2*np.pi,2*np.pi, 2*np.pi, 2*np.pi]),
                        dtype = float
                    ),
                #"stepnumber": spaces.Box(low = 0 , high = 500, dtype= int),
                "d_goal": spaces.Box(0,2,dtype=float),
                "dr_goal": spaces.Box(0,np.pi,dtype=float),
                                                   
            }
        )

        #further Parameters
        self.current_step = 0
        self.theta = np.zeros(6)
        self.goal = np.array([1,0.2,0.3])
        self.dist = 1
        self.crashed = False
        self.done = False
        self.theta_dot = np.zeros(6)

    def get_observation(self):
        #### todos: 
        # get motor,angles theta
        self.theta, self.theta_dot = FW.get_motor_pos()
        # get foreward kinematics 
        pose_ee = FW.get_forward_kinematics(self.theta)
        p_ee = np.array(pose_ee[:3,3])
        rot= R.from_matrix(pose_ee[:3,:3])
        rot_quat= np.array(rot.as_quat())
        #print("goal", self.goal,"p_ee", p_ee)
        self.dist, self.rot_dist = self.get_distance(self.goal, p_ee, rot)
        
        observation = { 
            "goal": self.goal,
            "p_ee": p_ee,
            "q_ee":rot_quat, 
            "theta": self.theta, 
            "d_goal": self.dist,
            "dr_goal": self.rot_dist 
        }
        
        return observation

    def get_distance(self, goal, p_ee, rot_ee):
        dist = np.linalg.norm(goal - p_ee)
        goal_rot = np.array([0,-1,0])
        ee_rot_quat = rot_ee #R.from_quat(rot_ee)
        ee_rot= ee_rot_quat.as_matrix()
        #rot_dist = np.arccos((np.trace(np.dot(goal_rot.T,ee_rot))-1)/2)
         #Compute the cosine similarity (dot product)
        y_axis = ee_rot[:, 1]  # Assuming 3x3 rotation matrix
        y_down = np.array([0,-1,0])  # downward direction for y 
        rot_dist = np.dot(y_axis, y_down) / (np.linalg.norm(y_axis) * np.linalg.norm(y_down))
        return np.array([dist]), np.array([np.absolute(rot_dist)])
        
    def get_reward(self): 
        R_success = 0 
        R_crash = 0 
        R_dist= self.dist ## distance to goal
        R_rot_dist = self.rot_dist 

        #successed
        if (self.dist <= 0.05):
            R_success = 1 
            print("successed, steps: ", self.current_step)
            self.done = True 
        # crashed
        if self.crashed: 
            print("crash")
            R_crash = 1
            self.done = True 
        # too many steps 
        if (self.current_step >= self.max_step):
            print("too many steps") 
            self.done = True 
            return 0  
        # total reward 
        total_reward = (-self.weights[0]*R_dist + 
            -self.weights[1]*R_rot_dist + 
            self.weights[2]*R_success + 
            -self.weights[3]*R_crash)     
        return total_reward[0]

         
    def step(self, action):
        
        #print("action",action)
        self.current_step += 1 
        #self.theta = FW.get_motor_pos()
        new_theta = np.clip(action, -3.13,3.13)
        FW.move_robot(new_theta)
        self.crashed = FW.check_crash()        
        observation = self.get_observation()
        reward = self.get_reward()

        truncated = False
        info = {}
        #print("reward",reward)

        return observation, reward, self.done, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)        
        #reset Initial pose:
        if self.crashed: 
           FW.reset_sim()   
        self.theta, self.theta_dot = FW.get_motor_pos()
        # new goal_pos
        rand_x = np.random.uniform(0.3, 0.6) 
        rand_y = np.random.uniform(-0.5, 0.5)
        rand_z = np.random.uniform(0.05,0.5)    
        self.goal = np.array([rand_x,rand_y,rand_z])
        FW.show_goal(self.goal)
            
        # reset parameters    
        self.crashed = False        
        self.done = False 
        self.current_step = 0
        observation = self.get_observation()
        #print("obs: " ,observation)
        info = {}
        
        return observation, info

    def render(self):
        pass

    def close(self):
        pass