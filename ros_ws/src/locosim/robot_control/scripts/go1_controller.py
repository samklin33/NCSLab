import time

import rospy as ros
import numpy as np
import base_controllers.params as conf

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for better compatibility
from matplotlib import pyplot as plt
from termcolor import colored
from base_controllers.quadruped_controller import QuadrupedController
from base_controllers.base_controller import BaseController
from base_controllers.utils.pidManager import PidManager
from base_controllers.utils.common_functions import *

robot = "go1"

class Go1Controller(QuadrupedController): 
    def __init__(self): 
        super().__init__(robot_name="go1")
        
        self.enable_realtime_plot = True
        self.plot_update_freq = 10  # Update plot every N control cycles
        self.plot_counter = 0

    def startupProcedure(self):
        # Force go1 to use the advanced startup procedure (not the simple BaseController one)
        ros.sleep(.5)
        print(colored("Starting Go1 advanced startup", "blue"))
        
        # Use the advanced startup from _startup_from_stand_down
        if self.go0_conf == 'standDown':
            self._startup_from_stand_down()
        else:
            self._startup_from_stand_up()
        
        print(colored("Go1 controller startup complete", "green"))
        
        # Setup real-time plotting after robot is initialized
        if self.enable_realtime_plot:
            self.setup_realtime_plot()

    def setup_realtime_plot(self):
        """Setup real-time plotting windows"""
        try:
            plt.ion()  # Turn on interactive mode
        
            # Create figure with subplots
            self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
            self.fig.suptitle('Go1 Robot Real-time Data', fontsize=16)
            
            # Initialize empty line objects for each leg
            self.leg_names = ['LF', 'RF', 'LH', 'RH']
            self.colors = ['red', 'blue', 'green', 'orange']
            
            # Subplot 1: Joint positions
            self.joint_lines = []
            ax_joints = self.axes[0, 0]
            for i in range(12):
                line, = ax_joints.plot([], [], label=f'Joint {i}')
                self.joint_lines.append(line)
            ax_joints.set_xlabel('Time (s)')
            ax_joints.set_ylabel('Joint Position (rad)')
            ax_joints.set_title('Joint Positions')
            ax_joints.grid(True)
            ax_joints.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            
            # Subplot 2: Contact forces
            self.force_lines = []
            ax_forces = self.axes[0, 1]
            for leg in range(4):
                line, = ax_forces.plot([], [], color=self.colors[leg], 
                                    label=f'{self.leg_names[leg]} leg')
                self.force_lines.append(line)
            ax_forces.set_xlabel('Time (s)')
            ax_forces.set_ylabel('Contact Force (N)')
            ax_forces.set_title('Contact Forces')
            ax_forces.grid(True)
            ax_forces.legend()
            
            # Subplot 3: Base position
            self.base_lines = []
            ax_base = self.axes[1, 0]
            base_labels = ['X', 'Y', 'Z']
            for i in range(3):
                line, = ax_base.plot([], [], label=f'Base {base_labels[i]}')
                self.base_lines.append(line)
            ax_base.set_xlabel('Time (s)')
            ax_base.set_ylabel('Position (m)')
            ax_base.set_title('Base Position')
            ax_base.grid(True)
            ax_base.legend()
            
            # Subplot 4: Foot heights
            self.foot_lines = []
            ax_feet = self.axes[1, 1]
            for leg in range(4):
                line, = ax_feet.plot([], [], color=self.colors[leg],
                                label=f'{self.leg_names[leg]} foot')
                self.foot_lines.append(line)
            ax_feet.set_xlabel('Time (s)')
            ax_feet.set_ylabel('Height (m)')
            ax_feet.set_title('Foot Heights')
            ax_feet.grid(True)
            ax_feet.legend()
            
            plt.tight_layout()
            plt.show(block=False)
            
            # Data storage for plotting
            self.plot_time_data = []
            self.plot_joint_data = [[] for _ in range(12)]
            self.plot_force_data = [[] for _ in range(4)]
            self.plot_base_data = [[] for _ in range(3)]
            self.plot_foot_data = [[] for _ in range(4)]
        
            # Keep only recent data (rolling window)
            self.max_plot_points = 500
            
        except Exception as e:
            print(colored(f"Warning: Could not setup real-time plotting: {e}", "yellow"))
            self.enable_realtime_plot = False

    def update_realtime_plot(self):
        """Update real-time plots with current data"""
        if not self.enable_realtime_plot:
            return
            
        try:
            # Update data arrays
            current_time = float(self.time[0]) if hasattr(self, 'time') else 0.0
            self.plot_time_data.append(current_time)
            
            # Joint positions
            for i in range(12):
                self.plot_joint_data[i].append(self.q[i])
                
            # Contact forces
            for leg in range(4):
                grf = self.u.getLegJointState(leg, self.grForcesW)
                force_magnitude = np.linalg.norm(grf)
                self.plot_force_data[leg].append(force_magnitude)
                
            # Base position
            for i in range(3):
                self.plot_base_data[i].append(self.basePoseW[i])
                
            # Foot heights
            for leg in range(4):
                self.plot_foot_data[leg].append(self.W_contacts[leg][2])
            
            # Keep only recent data (rolling window)
            if len(self.plot_time_data) > self.max_plot_points:
                self.plot_time_data = self.plot_time_data[-self.max_plot_points:]
                for i in range(12):
                    self.plot_joint_data[i] = self.plot_joint_data[i][-self.max_plot_points:]
                for leg in range(4):
                    self.plot_force_data[leg] = self.plot_force_data[leg][-self.max_plot_points:]
                    self.plot_foot_data[leg] = self.plot_foot_data[leg][-self.max_plot_points:]
                for i in range(3):
                    self.plot_base_data[i] = self.plot_base_data[i][-self.max_plot_points:]
            
            # Update line data
            time_array = np.array(self.plot_time_data)
            
            # Update joint lines
            for i in range(12):
                self.joint_lines[i].set_data(time_array, self.plot_joint_data[i])
                
            # Update force lines
            for leg in range(4):
                self.force_lines[leg].set_data(time_array, self.plot_force_data[leg])
                
            # Update base lines
            for i in range(3):
                self.base_lines[i].set_data(time_array, self.plot_base_data[i])
                
            # Update foot lines
            for leg in range(4):
                self.foot_lines[leg].set_data(time_array, self.plot_foot_data[leg])
            
            # Rescale axes
            for ax in self.axes.flat:
                ax.relim()
                ax.autoscale_view()
        
            # Redraw
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            print(colored(f"Warning: Plot update failed: {e}", "yellow"))
            # Don't disable plotting, just skip this update

    def control_loop(self, start_time): 
        """Main control loop for the Go1 robot"""
        # read robot state
        self.updateKinematics()

        # compute desired state
        self.tau_ffwd, self.grForcesW_des = self.wbc.gravityCompensationBase(self.B_contacts, self.wJ, self.h_joints, self.basePoseW)

        # send commands to robot
        current_time = time.time()
        if (current_time - start_time) % 2.0 < 0.01:
            print(colored(f"q_des: {np.round(self.q_des, 3)}\t", "yellow"))
            print(colored(f"qd_des: {np.round(self.qd_des, 3)}\t", "yellow"))
            print(colored(f"tau_ffwd: {np.round(self.tau_ffwd, 3)}\n", "yellow"))
        self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd, clip_commands=self.real_robot)
        
        # log data
        self.logData()    
        
        # visualize using RViz
        for leg in range(4):
            self.ros_pub.add_arrow(self.W_contacts[leg], self.contact_state[leg] * self.u.getLegJointState(leg, self.grForcesW/(6 * self.robot.robotMass)),"green")
            if (self.use_ground_truth_contacts):
                self.ros_pub.add_arrow(self.W_contacts[leg], self.u.getLegJointState(leg, self.grForcesW_gt / (6 * self.robot.robotMass)), "red")
        self.ros_pub.publishVisual()
        
        # Update real-time plots
        self.plot_counter += 1
        if self.plot_counter % self.plot_update_freq == 0:
            self.update_realtime_plot()

def talker(controller): 
    print(colored("Starting QuadrupedController...", "green"))
    
    # Use QuadrupedController's built-in startup method
    controller.startController(world_name='slow.world', use_ground_truth_pose=True, use_ground_truth_contacts=False, additional_args=['gui:=true', 'go0_conf:=standDown'])
    
    print(colored("Running startup procedure...", "green"))
    controller.startupProcedure()

    print(colored("Starting control loop...", "green"))
    rate = ros.Rate(1 / conf.robot_params[robot]['dt'])
    start_time = time.time()
    while not ros.is_shutdown():
        controller.control_loop(start_time)
        rate.sleep()
        controller.time = np.round(controller.time + np.array([controller.loop_time]), 4)

if __name__ == "__main__":
    ros.init_node('go1_controller')
    controller = Go1Controller()

    try:
        talker(controller)
    except KeyboardInterrupt:
        print("Shutting down controller...")
        ros.signal_shutdown("killed")
        controller.deregister_node()

    except Exception as e:
        print(f"Error occurred: {e}")
        ros.signal_shutdown("killed")
        controller.deregister_node()

    except (ros.ROSInterruptException, ros.service.ServiceException):
        print("Shutting down controller...")
        ros.signal_shutdown("killed")
        controller.deregister_node()