#!/usr/bin/env python3

import rospy
from haptiquad_plot.plotter_base import PlotterBase
from haptiquad_plot.plot_libs import PlotContainer
from haptiquad_plot.force_dialogs import SaveDialog
import tkinter as tk
from tkinter import ttk
import numpy as np

from haptiquad_msgs.msg import EstimatedForces
from geometry_msgs.msg import WrenchStamped
from anymal_msgs.msg import AnymalState

from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from copy import deepcopy


force_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
comp_labels = ["GT X", "EST X", "GT Y", "EST Y", "GT Z", "EST Z"]
norm_label = ["", "", "Norm Error", "", "", ""]
comp_colors = ['#1f77b4', '#14425f', '#ff7f0e', '#b0421e', '#2ca02c', '#4e781e']
comp_style = ['-', '--', '-', '--', '-', '--']

xlabels = ['Time [s]', 'Time [s]', 'Time [s]', 'Time [s]', 'Time [s]', 'Time [s]']
ylabels = ['Force [N]', 'Force [N]', 'Force [N]', 'Force [N]', 'Force [N]', 'Torque [Nm]']


class ForcePlotter(PlotterBase):

    def __init__(self):

        PlotterBase.__init__(self, "ForcePlotter")

        # Get foot_suffix parameter
        self.foot_suffix = rospy.get_param('~foot_suffix', "")

        if self.foot_suffix == "":
            raise RuntimeError("Foot suffix not set")

        mujoco_error = False
        anymal_error = False
        gazebo_error = False

        self.prev_mode = 0
        self.changed_axis = False
        self.prev_axis = 0
        self.foot_names = []
        self.forces, self.gt, self.gt_est, self.err, self.rmse, self.time, self.norm = {}, {}, {}, {}, {}, {}, {}
        
        
        for pref in self.legs_prefix:

            key = pref + '_' + self.foot_suffix
            self.forces[key] = np.empty((6, 0))
            self.gt[key] = np.empty((6, 0))
            self.gt_est[key] = np.empty((6, 0))
            self.err[key] = np.empty((6, 0))
            self.rmse[key] = np.empty((6, 0))
            self.time[key] = np.empty(0)
            self.norm[key] = np.empty(0)
            self.foot_names.append(key)

        for key in ["base_force", "base_torque"]:
            self.forces[key] = np.empty((6, 0))
            self.gt[key] = np.empty((6, 0))
            self.gt_est[key] = np.empty((6, 0))
            self.err[key] = np.empty((6, 0))
            self.rmse[key] = np.empty((6, 0))
            self.time[key] = np.empty(0)
            self.norm[key] = np.empty(0)
            self.foot_names.append(key)

        
        # ROS1 subscribers using message_filters
        self.est_sub = Subscriber('/estimated_forces', EstimatedForces)
        self.bag_sub = Subscriber('/state_estimator/anymal_state', AnymalState)
        
        self.bag_sync = TimeSynchronizer([self.bag_sub, self.est_sub], queue_size=10)
        self.bag_sync.registerCallback(self.bag_callback)
    
        self.init_from_params()


    def on_resize(self, event):

        # Update to get the current values
        self.update()
        curr_size = (self.winfo_width(), self.winfo_height())

        if self.previous_size != curr_size:

            height = self.winfo_height() / 2 - 50
            width = self.winfo_width() / 3 - 20

            for plot in self.plots.values():

                plot.canvas.get_tk_widget().config(width=width, height=height)
                plot.fast_update()

        self.previous_size = curr_size

    def add_GUI(self):
        

        self.show_torques = tk.BooleanVar()

        self.show_torques_butt = ttk.Checkbutton(self, style='Toggle.TButton', text="Show Torques", variable=self.show_torques, width=7)
        self.show_torques_butt.grid(row=0, column=7, padx=(5, 5), pady=10, sticky="we")

        ttk.Separator(self, orient=tk.VERTICAL).grid(row=0, column=8, sticky="ns", pady=5, padx=1)

        self.mode = tk.IntVar()
        self.mode.set(5)

        ttk.Label(self, text="Mode:").grid(row=0, column=9, sticky="ns", pady=5, padx=(0, 0))

        self.show_est_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Estimate", variable=self.mode, width=4, value=0)
        self.show_est_butt.grid(row=0, column=10, padx=(5, 5), pady=10, sticky="we")

        self.show_gt_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="GT", variable=self.mode, width=4, value=1)
        self.show_gt_butt.grid(row=0, column=11, padx=(5, 5), pady=10, sticky="we")

        self.show_err_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Error", variable=self.mode, width=4, value=2)
        self.show_err_butt.grid(row=0, column=12, padx=(5, 5), pady=10, sticky="we")

        self.show_mse_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="RMSE", variable=self.mode, width=4, value=3)
        self.show_mse_butt.grid(row=0, column=13, padx=(5, 5), pady=10, sticky="we")

        self.show_norm_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="Norm Err", variable=self.mode, width=4, value=4)
        self.show_norm_butt.grid(row=0, column=14, padx=(5, 5), pady=10, sticky="we")

        self.show_both_butt = ttk.Radiobutton(self, style='Toggle.TButton', text="GT+EST", variable=self.mode, width=4, value=5)
        self.show_both_butt.grid(row=0, column=15, padx=(5, 5), pady=10, sticky="we")

        self.only_z = tk.BooleanVar()
        self.only_z.set(False)
        self.only_z_butt = ttk.Checkbutton(self, text="Only Z", variable=self.only_z)
        self.only_z_butt.grid(row=0, column=16, padx=(5, 5), pady=10, sticky="we")

        self.plots = {
            self.legs_prefix[0]: PlotContainer(self, f"Estimated Forces - {self.legs_prefix[0]}"),
            self.legs_prefix[1]: PlotContainer(self, f"Estimated Forces - {self.legs_prefix[1]}"),
            self.legs_prefix[2]: PlotContainer(self, f"Estimated Forces - {self.legs_prefix[2]}"),
            self.legs_prefix[3]: PlotContainer(self, f"Estimated Forces - {self.legs_prefix[3]}"),
            "base_force": PlotContainer(self, f"Estimated Forces - Base Force"),
            "base_torque": PlotContainer(self, f"Estimated Forces - Base Torque"),
        }

        self.plots[self.legs_prefix[0]].grid(row=1, column=0, padx=0, pady=0, columnspan=6, sticky="w")
        self.plots[self.legs_prefix[1]].grid(row=2, column=0, padx=0, pady=0, columnspan=6, sticky="w")
        self.plots[self.legs_prefix[2]].grid(row=1, column=6, padx=0, pady=0, columnspan=7, sticky="w")
        self.plots[self.legs_prefix[3]].grid(row=2, column=6, padx=0, pady=0, columnspan=7, sticky="w")
        self.plots["base_force"].grid(row=1, column=13, padx=0, pady=0, columnspan=8, sticky="e")
        self.plots["base_torque"].grid(row=2, column=13, padx=0, pady=0, columnspan=8, sticky="e")


    def start_listening(self):
        
        self.listening = not self.listening
        self.show_torques_butt.config(state=tk.ACTIVE)    

        if self.listening:

            for plot in self.plots.values():

                plot.adjust_plots(0.15, 0.144, 0.938, 0.88, 0.2, 0.165)
                plot.clear()

            for pref in self.legs_prefix:

                key = pref + '_' + self.foot_suffix
                self.forces[key] = np.empty((6, 0))
                self.gt[key] = np.empty((6, 0))
                self.gt_est[key] = np.empty((6, 0))
                self.err[key] = np.empty((6, 0))
                self.rmse[key] = np.empty((6, 0))
                self.time[key] = np.empty(0)
                self.norm[key] = np.empty(0)

            for key in ["base_force", "base_torque"]:
                self.forces[key] = np.empty((6, 0))
                self.gt[key] = np.empty((6, 0))
                self.gt_est[key] = np.empty((6, 0))
                self.err[key] = np.empty((6, 0))
                self.rmse[key] = np.empty((6, 0))
                self.time[key] = np.empty(0)
                self.norm[key] = np.empty(0)

            self.show_torques_butt.config(state=tk.DISABLED)    

    def bag_callback(self, gt, est):
        
        if not self.listening:
            return

        for c in gt.contacts:

            f = np.array([
                    c.wrench.force.x,    
                    c.wrench.force.y,    
                    c.wrench.force.z,    
                    c.wrench.torque.x,    
                    c.wrench.torque.y,    
                    c.wrench.torque.z    
                ]).reshape((6, 1)) 

            self.gt[c.name] = np.hstack((self.gt[c.name], f))
            self.gt_est[c.name] = np.hstack((self.gt_est[c.name], np.zeros((6, 1))))
            self.gt_est[c.name][0][-1] = f[0][0]
            self.gt_est[c.name][2][-1] = f[1][0]
            self.gt_est[c.name][4][-1] = f[2][0]

            if self.gt[c.name].shape[1] > self.limit:
                self.gt[c.name] = self.gt[c.name][:, 1:]

            # ROS1 time conversion
            t = c.header.stamp.to_sec()
            if self.start_time is None:
                self.start_time = t            
            
            normalized_time = t - self.start_time
            self.time[c.name] = np.append(self.time[c.name], normalized_time)
            

        wrench = np.zeros((6, 1))
        self.gt["base_force"] = np.hstack((self.gt["base_force"], wrench))
        self.gt["base_torque"] = np.hstack((self.gt["base_torque"], wrench))
        self.gt_est["base_force"] = np.hstack((self.gt_est["base_force"], np.zeros((6, 1))))
        self.gt_est["base_torque"] = np.hstack((self.gt_est["base_torque"], np.zeros((6, 1))))
        
        t = gt.contacts[0].header.stamp.to_sec()
        normalized_time = t - self.start_time
        self.time["base_force"] = np.append(self.time["base_force"], normalized_time)
        self.time["base_torque"] = np.append(self.time["base_torque"], normalized_time)

        if self.gt["base_force"].shape[1] > self.limit:
                self.gt["base_force"] = self.gt["base_force"][:, 1:]
        if self.gt["base_torque"].shape[1] > self.limit:
                self.gt["base_torque"] = self.gt["base_torque"][:, 1:]

        for j in range(4):

            force = np.array([    
                    est.forces[j].force.x,
                    est.forces[j].force.y,
                    est.forces[j].force.z,
                    est.forces[j].torque.x,
                    est.forces[j].torque.y,
                    est.forces[j].torque.z,
                ]).reshape((6, 1))
            
            self.forces[est.names[j]] = np.hstack((self.forces[est.names[j]], force))
            self.gt_est[est.names[j]][1][-1] = force[0][0]
            self.gt_est[est.names[j]][3][-1] = force[1][0]
            self.gt_est[est.names[j]][5][-1] = force[2][0]
            if self.forces[est.names[j]].shape[1] > self.limit:
                self.forces[est.names[j]] = self.forces[est.names[j]][:, 1:]
                self.gt_est[est.names[j]] = self.gt_est[est.names[j]][:, 1:]


        base_force = np.array([    
                    est.forces[4].force.x,
                    est.forces[4].force.y,
                    est.forces[4].force.z,
                    0.0,
                    0.0,
                    0.0,
                ]).reshape((6, 1))
        
        base_torque = np.array([    
                    est.forces[4].torque.x,
                    est.forces[4].torque.y,
                    est.forces[4].torque.z,
                    0.0,
                    0.0,
                    0.0,
                ]).reshape((6, 1))
        
        self.forces["base_force"] = np.hstack((self.forces["base_force"], base_force))
        self.gt_est["base_force"][1][-1] = base_force[0][0]
        self.gt_est["base_force"][3][-1] = base_force[1][0]
        self.gt_est["base_force"][5][-1] = base_force[2][0]
        if self.forces["base_force"].shape[1] > self.limit:
            self.forces["base_force"] = self.forces["base_force"][:, 1:]
            self.gt_est["base_force"] = self.gt_est["base_force"][:, 1:]


        self.forces["base_torque"] = np.hstack((self.forces["base_torque"], base_torque))
        self.gt_est["base_torque"][1][-1] = base_torque[0][0]
        self.gt_est["base_torque"][3][-1] = base_torque[1][0]
        self.gt_est["base_torque"][5][-1] = base_torque[2][0]
        if self.forces["base_torque"].shape[1] > self.limit:
            self.forces["base_torque"] = self.forces["base_torque"][:, 1:]
            self.gt_est["base_torque"] = self.gt_est["base_torque"][:, 1:]

        self.update_stats()
        


    def update_stats(self):

        for i in range(6):

            key = self.foot_names[i]

            self.err[key] = self.gt[key] - self.forces[key]

            mse = np.mean((self.gt[key] - self.forces[key])**2, axis=1).reshape((6, 1))

            self.rmse[key] = np.hstack((self.rmse[key], np.sqrt(mse)))

            self.norm[key] = np.append(self.norm[key], np.linalg.norm(self.gt[key][0:2, -1] - self.forces[key][0:2, -1]))

            if self.rmse[key].shape[1] > self.limit:

                self.rmse[key] = self.rmse[key][:, 1:]
                self.norm[key] = self.norm[key][1:]        
                self.time[key] = self.time[key][1:]



    def process_mode(self, i, mode):

        key = self.foot_names[i]
        labels = force_labels
        colors = None        
        style = None
        pause = self.pause.get()
        time = self.time[key] if not pause else self.frozen_data['time'][key]
        xlabel = xlabels[i]
        ylabel = ylabels[i]
        if i < 4:
            prefix = self.legs_prefix[i]
        elif i == 4:
            prefix = "Base Force"
        elif i == 5:
            prefix = "Base Torque"

        if mode == 0:
            
            to_plot = self.forces[key] if not pause else self.frozen_data['forces'][key]
            title = f'Estimated forces - {prefix}'

        elif mode == 1:

            to_plot = self.gt[key] if not pause else self.frozen_data['gt'][key]
            title = f'Groundtruth forces - {prefix}'

        elif mode == 2:

            to_plot = self.err[key] if not pause else self.frozen_data['error'][key]
            title = f'Estimation error - {prefix}'

        elif mode == 3:

            to_plot = self.rmse[key] if not pause else self.frozen_data['rmse'][key]            
            title = f'RMSE - {prefix}'

        elif mode == 4:

            to_plot = self.norm[key] if not pause else self.frozen_data['norm'][key]
            l = to_plot.shape[0]
            to_plot = np.array([np.zeros(l), np.zeros(l), to_plot])        
            labels = norm_label
            title = f'Norm error - {prefix}'

        elif mode == 5:

            to_plot = self.gt_est[key] if not pause else self.frozen_data['comp'][key]
            title = f'Forces comparison - {prefix}'
            colors = comp_colors
            labels = comp_labels
            style = comp_style

            if self.only_z.get() and not self.show_torques.get():

                to_plot = to_plot[-2:, :]
                labels = comp_labels[-2:]

        if not self.show_torques.get() and not self.only_z.get() and not mode == 5:

            to_plot = to_plot[:3, :]
            labels = labels[:3]

        if self.only_z.get() and not mode in [4, 5]:

            l = to_plot[2, :].shape
            to_plot = np.array([np.zeros(l), np.zeros(l), to_plot[2, :]])
            labels = labels[:3]


        return to_plot, title, labels, colors, time, style, xlabel, ylabel
            

    
    def update_plots(self):


        if self.only_z.get() != self.prev_axis:
            self.prev_axis = self.only_z.get()
            self.changed_axis = True

        if self.prev_mode != self.mode.get() or self.changed_axis:
            self.prev_mode = self.mode.get()
            self.changed_axis = False
            for plot in self.plots.values():
                plot.clear()

        if self.pause.get() and type(self.frozen_data) == type(None):
            
            self.frozen_data = {}
            self.frozen_data['forces'] = deepcopy(self.forces)
            self.frozen_data['gt'] = deepcopy(self.gt)
            self.frozen_data['error'] = deepcopy(self.err)
            self.frozen_data['rmse'] = deepcopy(self.rmse)
            self.frozen_data['norm'] = deepcopy(self.norm)
            self.frozen_data['comp'] = deepcopy(self.gt_est)
            self.frozen_data['time'] = deepcopy(self.time)

        elif not self.pause.get() and type(self.frozen_data) == dict:

            del self.frozen_data
            self.frozen_data = None

        mode = self.mode.get()


        for i, plot in enumerate(self.plots.values()):        


            to_plot, title, labels, colors, time, style, xlabel, ylabel = self.process_mode(i, mode)            

            if not time.shape[0] == to_plot.shape[1]:

                continue

            plot.update_plot(to_plot, labels, title=title, xlabel=xlabel, ylabel=ylabel, time=time, color=colors, style=style)

    def save_plots(self):

        SaveDialog(self, self.frozen_data)


def main(args=None):
    plotter = ForcePlotter()
    plotter.run()


if __name__ == '__main__':
    main()