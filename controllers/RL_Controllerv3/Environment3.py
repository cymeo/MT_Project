import From_Webot3 as FW
from gymnasium import Env
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation as R

class WeBot_environment(Env):
    """Custom Environment that follows gym interface."""
    # action: choose motorspeed for each motor
    # observation: goal pose, endeffector pose, robot motor angles,  goal distance, arm position and arm speed 

    def __init__(self):
        ######### rewards for -distance to goal,-rotational_distance, success, -max_steps, crash ############
        self.weights = np.array([0.1,0.03,5,50]) 
        self.max_step = 700
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
                "q_goal": spaces.Box(
                    low = np.array([-1,-1,-1,-1]),
                    high =np.array([1,1,1,1]), 
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
                "dr_goal": spaces.Box(0,2*np.pi,dtype=float),
                "p_arm": spaces.Box(
                    low = np.array([0,0,0]),
                    high =np.array([1.5,1.5,1.5]), 
                    dtype = float
                    ), 
                       
                 "v_arm": spaces.Box(
                    low = np.array([0,0,0]),
                    high =np.array([2,2,2]), 
                    dtype = float
                    ), 
                 "d_arm": spaces.Box(0,4,dtype=float),                              
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
        self.q_goal = np.array([0,0,0,0]) 
        
        
    def get_observation(self):
        #### todos: 
        # get motor,angles theta
        self.theta, self.theta_dot = FW.get_motor_pos()
        # get end effector information  
        pose_ee = FW.get_forward_kinematics(self.theta)
        p_ee = np.array(pose_ee[:3,3])
        rot_ee= R.from_matrix(pose_ee[:3,:3])
        q_ee= np.array(rot_ee.as_quat())
        
        
        # get distance to goal, slightly unneccesarly, learns faster though   
        # linear distance 
        self.dist = np.array([np.linalg.norm(self.goal - p_ee)])
        #rotational distance 
        rot_goal = R.from_quat(self.q_goal)
        rot_diff = rot_goal.inv() * rot_ee        
        rot_dist = 2 * np.arccos(np.clip(rot_diff.as_quat()[-1], -1.0, 1.0))/ (2*np.pi) # normed distance 
        self.rot_dist= np.array([np.abs(rot_dist)])
        
        # get armobject 
        p_arm, v_arm = FW.get_arm()
        d_arm =  np.array([np.linalg.norm(p_ee-p_arm)])

        observation = { 
            "goal": self.goal,
            "q_goal": self.q_goal, 
            "p_ee": p_ee,
            "q_ee": q_ee, 
            "theta": self.theta, 
            "d_goal": self.dist,
            "dr_goal": self.rot_dist,
            "p_arm": p_arm, 
            "v_arm": v_arm,    
            "d_arm": d_arm
        }
        return observation

        
    def get_reward(self): 
        R_success = 0 
        R_crash = 0 
        R_dist= self.dist ## distance to goal
        R_rot_dist = self.rot_dist 

        #successed
        if (self.dist <= 0.03):
            R_success = 1 
            print("successed, steps: ", self.current_step)
            self.done = True 
            print("rot_dist",self.rot_dist)
            if self.rot_dist<= 0.01: 
                print("rot success")
                R_success += 0.5
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
        self.current_step += 1 
        #self.theta = FW.get_motor_pos()
        new_theta = np.clip(action, -np.pi/2, np.pi/2)
        FW.move_robot(new_theta)
        self.crashed = FW.check_crash()        
        observation = self.get_observation()
        reward = self.get_reward()

        truncated = False
        info = {}
        
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
        rand_z = np.random.uniform(0.05,0.4)    
        self.goal = np.array([rand_x,rand_y,rand_z])
        R_zdown = [[0,-1,0], [-1,0,0], [0,0,-1]]
        self.q_goal =  R.from_matrix(R_zdown).as_quat()
        FW.show_goal(self.goal, self.q_goal)
            
        # reset parameters    
        self.crashed = False        
        self.done = False 
        self.current_step = 0
        observation = self.get_observation()
        info = {}
        
        return observation, info

    def render(self):
        pass

    def close(self):
        pass