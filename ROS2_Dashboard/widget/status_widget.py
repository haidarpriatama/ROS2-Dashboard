import tkinter as tk
from tkinter import ttk

def create_status_widget(dashboard):
    """Create the status widget"""
    # Status widget
    dashboard.status_frame = tk.LabelFrame(dashboard.main_frame, text="STATUS", padx=5, pady=5)
    dashboard.status_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
    
    # Create frame for two robot status
    dashboard.robots_frame = tk.Frame(dashboard.status_frame)
    dashboard.robots_frame.pack(fill=tk.BOTH, expand=True)
    
    # Robot 1 status
    dashboard.robot1_frame = tk.LabelFrame(dashboard.robots_frame, text="ROBOT 1", padx=5, pady=2)
    dashboard.robot1_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=2, rowspan=2)
    
    # Robot 1 status parameters - now including CPU Temperature
    dashboard.robot1_params = {}
    status_params = [
        ("CPU Usage", "cpu_usage_1"),
        ("Memory Usage", "memory_usage_1"), 
        ("CPU Temp", "cpu_temp_1"),
        ("Battery", "battery_1")
    ]
    
    for idx, (label_text, attr_name) in enumerate(status_params):
        tk.Label(dashboard.robot1_frame, text=label_text, anchor="e", width=15).grid(row=idx, column=0, sticky="e", pady=2)
        tk.Label(dashboard.robot1_frame, text=":").grid(row=idx, column=1, padx=5)
        # Create a frame to hold the value and unit symbol together
        value_frame = tk.Frame(dashboard.robot1_frame)
        value_frame.grid(row=idx, column=2, sticky="w")
        
        # Create the value label in the frame
        setattr(dashboard, attr_name, tk.Label(value_frame, text="N/A", width=4, anchor="e"))
        getattr(dashboard, attr_name).pack(side=tk.LEFT, padx=0)
        
        # Create unit symbol in the same frame
        if "temp" in attr_name:
            # Create degree Celsius symbol for temperature
            setattr(dashboard, f"{attr_name}_unit", tk.Label(value_frame, text="°C"))
            getattr(dashboard, f"{attr_name}_unit").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_unit").pack_forget()  # Hide initially
        else:
            # Create percentage symbol for other parameters
            setattr(dashboard, f"{attr_name}_pct", tk.Label(value_frame, text="%"))
            getattr(dashboard, f"{attr_name}_pct").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_pct").pack_forget()  # Hide initially
        
        dashboard.robot1_params[attr_name] = "N/A"
    
    # Robot 2 status
    dashboard.robot2_frame = tk.LabelFrame(dashboard.robots_frame, text="ROBOT 2", padx=5, pady=2)
    dashboard.robot2_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=2, rowspan=2)
    
    # Robot 2 status parameters - now including CPU Temperature
    dashboard.robot2_params = {}
    status_params = [
        ("CPU Usage", "cpu_usage_2"), 
        ("Memory Usage", "memory_usage_2"), 
        ("CPU Temp", "cpu_temp_2"),
        ("Battery", "battery_2")
    ]
    
    for idx, (label_text, attr_name) in enumerate(status_params):
        tk.Label(dashboard.robot2_frame, text=label_text, anchor="e", width=15).grid(row=idx, column=0, sticky="e", pady=2)
        tk.Label(dashboard.robot2_frame, text=":").grid(row=idx, column=1, padx=5)
        # Create a frame to hold the value and unit symbol together
        value_frame = tk.Frame(dashboard.robot2_frame)
        value_frame.grid(row=idx, column=2, sticky="w")
        
        # Create the value label in the frame
        setattr(dashboard, attr_name, tk.Label(value_frame, text="N/A", width=4, anchor="e"))
        getattr(dashboard, attr_name).pack(side=tk.LEFT, padx=0)
        
        # Create unit symbol in the same frame
        if "temp" in attr_name:
            # Create degree Celsius symbol for temperature
            setattr(dashboard, f"{attr_name}_unit", tk.Label(value_frame, text="°C"))
            getattr(dashboard, f"{attr_name}_unit").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_unit").pack_forget()  # Hide initially
        else:
            # Create percentage symbol for other parameters
            setattr(dashboard, f"{attr_name}_pct", tk.Label(value_frame, text="%"))
            getattr(dashboard, f"{attr_name}_pct").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_pct").pack_forget()  # Hide initially
        
        dashboard.robot2_params[attr_name] = "N/A"
    
    # Battery progress bars
    # Create a style for green horizontal progress bars
    style = ttk.Style()
    style.theme_use('default')
    style.configure("green.Horizontal.TProgressbar", troughcolor='white', background='green')
    
    # Add horizontal battery progress bars directly in robot frames
    dashboard.battery1_progress = ttk.Progressbar(
        dashboard.robot1_frame, 
        orient=tk.HORIZONTAL, 
        length=150, 
        mode='determinate', 
        style="green.Horizontal.TProgressbar"
    )
    dashboard.battery1_progress.grid(row=4, column=0, columnspan=3, pady=5, padx=5, sticky="ew")
    
    dashboard.battery2_progress = ttk.Progressbar(
        dashboard.robot2_frame, 
        orient=tk.HORIZONTAL, 
        length=150, 
        mode='determinate', 
        style="green.Horizontal.TProgressbar"
    )
    dashboard.battery2_progress.grid(row=4, column=0, columnspan=3, pady=5, padx=5, sticky="ew")
    
    # Configure robot frames to expand
    dashboard.robots_frame.grid_columnconfigure(0, weight=1)
    dashboard.robots_frame.grid_columnconfigure(1, weight=1)