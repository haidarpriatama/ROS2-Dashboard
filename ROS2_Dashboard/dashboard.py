#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from threading import Thread
import rclpy
from rclpy.node import Node
import time
import os

# Import GUI components
from widget.data_logger_widget import create_data_logger_widget
from widget.send_data_widget import create_send_data_widget
from widget.status_widget import create_status_widget
from widget.visual_widget import create_visual_widget, add_graph_methods
from widget.gui_info_widget import create_gui_info_widget

# Import utilities
from utils.config_manager import load_config, save_config
from utils.topic_manager import (
    get_available_topics, 
    subscribe_to_topic, 
    subscribe_to_status, 
    subscribe_to_battery, 
    subscribe_to_position,
    subscribe_to_graph_topic,
    update_robot_position,
    send_data
)

class ROS2_Dashboard:
    def __init__(self, master):
        self.master = master
        self.master.title("ROS2 Dashboard")
        self.master.wm_title("ROS Dashboard") 
        self.master.geometry("1700x850")
        # self.master.resizable(False, False)
        
        # Set logo icon
        icon = ImageTk.PhotoImage(Image.open("/home/haidar/ros2_git/src/ROS2-Dashboard/ROS2_Dashboard/resources/icon_app.png"))
        self.master.iconphoto(True, icon)
        
        # Config file path
        self.config_file = os.path.join(os.path.expanduser("~"), ".ros2_dashboard_config.json")
        
        # ROS2 Node initialization
        self.node = Node("ros2_dashboard_gui")
        
        # Subscribed topics (now supporting 6 topics)
        self.topic1 = None
        self.topic2 = None
        self.topic3 = None
        self.topic4 = None
        self.topic5 = None
        self.topic6 = None
        self.status_topic1 = None  # For Robot 1
        self.status_topic2 = None  # For Robot 2
        self.output_topic = None
        self.graph_topic = None  # New graph topic
        
        # Topic data (now supporting 6 topics)
        self.topic1_data = []
        self.topic2_data = []
        self.topic3_data = []
        self.topic4_data = []
        self.topic5_data = []
        self.topic6_data = []
        self.status_data = ["N/A", "N/A", "N/A", "N/A"]
        
        # Graph-related data attributes
        self.graph_time_data = []
        self.graph_x_data = []
        self.graph_y_data = []
        self.graph_spx_data = []
        self.graph_spy_data = []
        self.graph_paused = False  # Initialize graph pause state
        
        # Flag for safe shutdown
        self.stopping = False
        
        # Available topics - get them first before creating UI
        self.available_topics = []
        self.available_types = {}
        self.get_available_topics()
        
        # Create main frame
        self.main_frame = tk.Frame(master)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Add graph methods
        add_graph_methods(self)
        
        # Create widgets
        create_data_logger_widget(self)
        create_send_data_widget(self)
        create_status_widget(self)
        create_visual_widget(self)
        create_gui_info_widget(self)
            
        # Load configuration
        self.load_config()
        
        # Start ROS2 spinner in a separate thread
        self.ros_thread = Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start clock update
        self.update_clock()
        
        # Start automatic topic refresh timer (every 5 seconds)
        self.auto_refresh_topics()
        
        # Configure grid weights
        self.main_frame.grid_rowconfigure(0, weight=0, minsize=250)  # Reduced height for Send Data and Status
        self.main_frame.grid_rowconfigure(1, weight=1)  # Row for Position
        self.main_frame.grid_columnconfigure(0, weight=5)  # Column for Data Logger
        self.main_frame.grid_columnconfigure(1, weight=1)  # Column for Send Data
        self.main_frame.grid_columnconfigure(2, weight=0)  # Column for Status
        self.main_frame.grid_columnconfigure(3, weight=0)  # Column for GUI Info
    
    def update_clock(self):
        # Update time and date
        current_time = time.strftime("%H:%M")
        current_date = time.strftime("%d/%m/%Y")
        self.time_label.config(text=current_time)
        self.date_label.config(text=current_date)
        self.master.after(1000, self.update_clock)  # Update every second
    
    def get_available_topics(self):
        """Use ROS2 API to get actual available topics."""
        return get_available_topics(self)
    
    def auto_refresh_topics(self):
        """Automatically refresh topics periodically."""
        # Get available topics
        old_topics = set(self.available_topics)
        self.get_available_topics()
        new_topics = set(self.available_topics)
        
        # If topics have changed, update all comboboxes
        if old_topics != new_topics:
            print("Topics list updated automatically")
            # Update all comboboxes with the new topic list
            for i in range(1, 7):  # Now supporting 6 topics
                combo_attr = f"topic{i}_combo"
                if hasattr(self, combo_attr):
                    current_value = getattr(self, combo_attr).get()
                    getattr(self, combo_attr)['values'] = self.available_topics
                    if current_value in self.available_topics:
                        getattr(self, combo_attr).set(current_value)
            
            # Update output topic combobox
            if hasattr(self, 'output_topics'):
                current_value = self.output_topics.get()
                self.output_topics['values'] = self.available_topics
                if current_value in self.available_topics:
                    self.output_topics.set(current_value)
            
            # Update position topic comboboxes
            for i in range(1, 3):
                combo_attr = f"pos_topic{i}_combo"
                if hasattr(self, combo_attr):
                    current_value = getattr(self, combo_attr).get()
                    getattr(self, combo_attr)['values'] = self.available_topics
                    if current_value in self.available_topics:
                        getattr(self, combo_attr).set(current_value)
            
            # Update graph topic combobox
            if hasattr(self, 'graph_topic_combo'):
                current_value = self.graph_topic_combo.get()
                self.graph_topic_combo['values'] = self.available_topics
                if current_value in self.available_topics:
                    self.graph_topic_combo.set(current_value)
        
        # Schedule the next refresh (every 5 seconds)
        self.master.after(5000, self.auto_refresh_topics)
    
    def open_settings(self):
        # Create settings window for all configurations
        settings_window = tk.Toplevel(self.master)
        settings_window.title("Global Settings")
        settings_window.geometry("400x350")  # Make window a bit smaller due to removed sections
        settings_window.grab_set()  # Make window modal
        
        # Create outer frame with fixed bottom panel for the Save button
        outer_frame = tk.Frame(settings_window)
        outer_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create a canvas with scrollbar for the content
        canvas = tk.Canvas(outer_frame)
        scrollbar = ttk.Scrollbar(outer_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        # Configure the canvas
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        # Bind mouse wheel for scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Keep track of the window in the instance for topic refresh
        self.settings_window = settings_window
        self.settings_combos = []
        
        # Topics configuration
        tk.Label(scrollable_frame, text="Global Configuration", font=("Arial", 12, "bold")).pack(pady=10)
    
        settings_frame = tk.Frame(scrollable_frame)
        settings_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        row = 0
        
        # Data logger topics configuration
        topic_label = tk.Label(settings_frame, text="Data Logger Topics", font=("Arial", 10, "bold"))
        topic_label.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        row += 1
        
        # Create 6 topic settings combos
        for i in range(1, 7):
            tk.Label(settings_frame, text=f"Topic {i}:").grid(row=row, column=0, sticky="w", pady=5)
            combo = ttk.Combobox(settings_frame, values=self.available_topics, width=30)
            if hasattr(self, f"topic{i}") and getattr(self, f"topic{i}"):
                combo.set(getattr(self, f"topic{i}"))
            combo.grid(row=row, column=1, padx=5, pady=5)
            setattr(self, f"settings_topic{i}_combo", combo)
            self.settings_combos.append(combo)
            row += 1
        
        # Position topics
        position_label = tk.Label(settings_frame, text="Robot Position Topics", font=("Arial", 10, "bold"))
        position_label.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        row += 1
    
        # Robot 1 position topic
        tk.Label(settings_frame, text="Robot 1 Position:").grid(row=row, column=0, sticky="w", pady=5)
        robot1_pos_combo = ttk.Combobox(settings_frame, values=self.available_topics, width=30)
        if hasattr(self, "pos_topic1") and self.pos_topic1:
            robot1_pos_combo.set(self.pos_topic1)
        robot1_pos_combo.grid(row=row, column=1, padx=5, pady=5)
        self.settings_robot1_pos_combo = robot1_pos_combo
        self.settings_combos.append(robot1_pos_combo)
        row += 1
        
        # Robot 2 position topic
        tk.Label(settings_frame, text="Robot 2 Position:").grid(row=row, column=0, sticky="w", pady=5)
        robot2_pos_combo = ttk.Combobox(settings_frame, values=self.available_topics, width=30)
        if hasattr(self, "pos_topic2") and self.pos_topic2:
            robot2_pos_combo.set(self.pos_topic2)
        robot2_pos_combo.grid(row=row, column=1, padx=5, pady=5)
        self.settings_robot2_pos_combo = robot2_pos_combo
        self.settings_combos.append(robot2_pos_combo)
        row += 1
        
        # Graph topic configuration
        graph_label = tk.Label(settings_frame, text="Graph Topic", font=("Arial", 10, "bold"))
        graph_label.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        row += 1
    
        # Graph topic selection
        tk.Label(settings_frame, text="Graph Topic:").grid(row=row, column=0, sticky="w", pady=5)
        graph_topic_combo = ttk.Combobox(settings_frame, values=self.available_topics, width=30)
        if hasattr(self, "graph_topic") and self.graph_topic:
            graph_topic_combo.set(self.graph_topic)
        graph_topic_combo.grid(row=row, column=1, padx=5, pady=5)
        self.settings_graph_topic_combo = graph_topic_combo
        self.settings_combos.append(graph_topic_combo)
        row += 1
        
        # Pack the canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Create bottom frame for Save button (fixed position)
        button_frame = tk.Frame(settings_window)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=10)
        
        # Save button
        tk.Button(button_frame, text="Save", command=lambda: self.save_settings(settings_window)).pack(pady=5)
        
        # Start auto refresh for the settings window
        self.refresh_settings_topics()
        
        # Bind the window destroy event to clean up refresh timer
        settings_window.protocol("WM_DELETE_WINDOW", self.on_settings_close)
    
    def save_settings(self, window):
        # Save all settings
        for i in range(1, 7):  # Now supporting 6 topics
            topic = getattr(self, f"settings_topic{i}_combo").get()
            if topic:
                self.subscribe_to_topic(i, topic)
                # Also update the combobox in main UI
                if hasattr(self, f"topic{i}_combo"):
                    getattr(self, f"topic{i}_combo").set(topic)
        
        # Auto-subscribe to status topics for both robots (always use predefined topics)
        for i in range(1, 3):
            status_topic = f"/ROBOT{i}/STATUS"
            if status_topic in self.available_topics:
                self.subscribe_to_status(status_topic, i)
            
            # Auto-subscribe to battery topics for both robots
            battery_topic = f"/ROBOT{i}/battery"
            if battery_topic in self.available_topics:
                self.subscribe_to_battery(battery_topic, i)
        
        # Subscribe to position topics for both robots
        robot1_pos_topic = self.settings_robot1_pos_combo.get()
        if robot1_pos_topic:
            self.subscribe_to_position(robot1_pos_topic, 1)

        robot2_pos_topic = self.settings_robot2_pos_combo.get()
        if robot2_pos_topic:
            self.subscribe_to_position(robot2_pos_topic, 2)
        
        # Subscribe to graph topic
        graph_topic = self.settings_graph_topic_combo.get()
        if graph_topic:
            self.subscribe_to_graph_topic(graph_topic)
            # Update graph topic combobox in main UI
            if hasattr(self, 'graph_topic_combo'):
                self.graph_topic_combo.set(graph_topic)
        
        # Save config to file
        self.save_config()
        
        # Close the settings window
        window.destroy()
    
    def refresh_settings_topics(self):
        """Refresh topics in the settings window"""
        # First, check if the settings window still exists
        if hasattr(self, 'settings_window') and self.settings_window.winfo_exists():
            # Get fresh topics
            old_topics = list(self.available_topics)
            self.get_available_topics()
            new_topics = list(self.available_topics)
            
            # Always update all comboboxes in settings to ensure they're current
            for combo in self.settings_combos:
                current_value = combo.get()
                combo['values'] = new_topics
                if current_value in new_topics:
                    combo.set(current_value)
            
            # Print debug message if topics changed
            if set(old_topics) != set(new_topics):
                print("Topics list updated in settings window")
            
            # Schedule the next refresh (every 1 second for more responsive updates)
            self.settings_refresh_id = self.master.after(1000, self.refresh_settings_topics)
        else:
            # Cleanup if window no longer exists
            if hasattr(self, 'settings_refresh_id'):
                self.master.after_cancel(self.settings_refresh_id)
                delattr(self, 'settings_refresh_id')
    
    def on_settings_close(self):
        """Handle settings window close event"""
        # Cancel the refresh timer if it exists
        if hasattr(self, 'settings_refresh_id'):
            self.master.after_cancel(self.settings_refresh_id)
            delattr(self, 'settings_refresh_id')
        
        # Unbind mouse wheel event
        self.master.unbind_all("<MouseWheel>")
        
        # Close the window
        if hasattr(self, 'settings_window'):
            self.settings_window.destroy()
            delattr(self, 'settings_window')
    
    def ros_spin(self):
        # ROS2 spin in a separate thread
        while rclpy.ok() and not self.stopping:
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def exit_application(self):
        # Save current configuration
        self.save_config()
        
        # Set a flag to stop the ROS thread
        self.stopping = True
        
        # Destroy all subscriptions first
        for attr_name in dir(self):
            if attr_name.endswith('_sub'):
                try:
                    self.node.destroy_subscription(getattr(self, attr_name))
                except:
                    pass
        
        # Destroy graph topic subscription if it exists
        if hasattr(self, 'graph_topic_sub'):
            try:
                self.node.destroy_subscription(self.graph_topic_sub)
            except:
                pass
        
        # Destroy all publishers
        for attr_name in dir(self):
            if attr_name.endswith('_pub') or attr_name == 'output_publisher':
                try:
                    self.node.destroy_publisher(getattr(self, attr_name))
                except:
                    pass
        
        # Use after to give time for cleanup and prevent freezing
        self.master.after(100, self._complete_shutdown)

    def _complete_shutdown(self):
        try:
            # Clean up ROS2 resources
            if hasattr(self, 'node'):
                self.node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        
        # Exit application
        self.master.quit()
        self.master.destroy()

# Import subscription functions and methods
ROS2_Dashboard.subscribe_to_topic = subscribe_to_topic
ROS2_Dashboard.subscribe_to_status = subscribe_to_status
ROS2_Dashboard.subscribe_to_battery = subscribe_to_battery
ROS2_Dashboard.subscribe_to_position = subscribe_to_position
ROS2_Dashboard.subscribe_to_graph_topic = subscribe_to_graph_topic
ROS2_Dashboard.update_robot_position = update_robot_position
ROS2_Dashboard.send_data = send_data
ROS2_Dashboard.save_config = save_config
ROS2_Dashboard.load_config = load_config