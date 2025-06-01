import tkinter as tk
from tkinter import ttk

# RPM scale constants for easy maintenance
MAX_RPM = 200
RPM_WARNING_THRESHOLD = 3000  # Level where color starts turning orange
RPM_CRITICAL_THRESHOLD = 3500  # Level where color turns red

def create_status_widget(dashboard):
    """Create the status widget"""
    # Status widget
    dashboard.status_frame = tk.LabelFrame(dashboard.main_frame, text="STATUS", padx=5, pady=5)
    dashboard.status_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
    
    # Create a notebook (tabbed interface)
    dashboard.status_notebook = ttk.Notebook(dashboard.status_frame)
    dashboard.status_notebook.pack(fill=tk.BOTH, expand=True)
    
    # Create tabs
    dashboard.vitals_tab = ttk.Frame(dashboard.status_notebook)
    dashboard.rpm_tab = ttk.Frame(dashboard.status_notebook)
    
    # Add tabs to notebook
    dashboard.status_notebook.add(dashboard.vitals_tab, text="Vitals")
    dashboard.status_notebook.add(dashboard.rpm_tab, text="RPM")
    
    # Create vitals content (the existing functionality)
    create_vitals_tab(dashboard)
    
    # Create RPM content (new functionality)
    create_rpm_tab(dashboard)

def create_vitals_tab(dashboard):
    """Create the vitals tab with CPU, memory, etc. info"""
    # Create frame for two robot status
    dashboard.robots_frame = tk.Frame(dashboard.vitals_tab)
    dashboard.robots_frame.pack(fill=tk.BOTH, expand=True)
    
    # Robot 1 status
    dashboard.robot1_frame = tk.LabelFrame(dashboard.robots_frame, text="ROBOT 1", padx=5, pady=2)
    dashboard.robot1_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=2, rowspan=2)
    
    # Robot 1 status parameters - now including Bandwidth
    dashboard.robot1_params = {}
    status_params = [
        ("CPU Usage", "cpu_usage_1"),
        ("Memory Usage", "memory_usage_1"), 
        ("CPU Temp", "cpu_temp_1"),
        ("Bandwidth", "bandwidth_1")  # Ganti Battery dengan Bandwidth
    ]
    
    for idx, (label_text, attr_name) in enumerate(status_params):
        tk.Label(dashboard.robot1_frame, text=label_text, anchor="e", width=15).grid(row=idx, column=0, sticky="e", pady=2)
        tk.Label(dashboard.robot1_frame, text=":").grid(row=idx, column=1, padx=5)
        # Create a frame to hold the value and unit symbol together
        value_frame = tk.Frame(dashboard.robot1_frame)
        value_frame.grid(row=idx, column=2, sticky="w")
        
        # Create the value label in the frame
        setattr(dashboard, attr_name, tk.Label(value_frame, text="N/A", width=8, anchor="e"))
        getattr(dashboard, attr_name).pack(side=tk.LEFT, padx=0)
        
        # Create unit symbol in the same frame
        if "temp" in attr_name:
            # Create degree Celsius symbol for temperature
            setattr(dashboard, f"{attr_name}_unit", tk.Label(value_frame, text="°C"))
            getattr(dashboard, f"{attr_name}_unit").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_unit").pack_forget()  # Hide initially
        elif "bandwidth" in attr_name:
            # Create Kb/s symbol for bandwidth
            setattr(dashboard, f"{attr_name}_unit", tk.Label(value_frame, text="Kb/s"))
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
    
    # Robot 2 status parameters - now including Bandwidth
    dashboard.robot2_params = {}
    status_params = [
        ("CPU Usage", "cpu_usage_2"), 
        ("Memory Usage", "memory_usage_2"), 
        ("CPU Temp", "cpu_temp_2"),
        ("Bandwidth", "bandwidth_2")  # Ganti Battery dengan Bandwidth
    ]
    
    for idx, (label_text, attr_name) in enumerate(status_params):
        tk.Label(dashboard.robot2_frame, text=label_text, anchor="e", width=15).grid(row=idx, column=0, sticky="e", pady=2)
        tk.Label(dashboard.robot2_frame, text=":").grid(row=idx, column=1, padx=5)
        # Create a frame to hold the value and unit symbol together
        value_frame = tk.Frame(dashboard.robot2_frame)
        value_frame.grid(row=idx, column=2, sticky="w")
        
        # Create the value label in the frame
        setattr(dashboard, attr_name, tk.Label(value_frame, text="N/A", width=8, anchor="e"))
        getattr(dashboard, attr_name).pack(side=tk.LEFT, padx=0)
        
        # Create unit symbol in the same frame
        if "temp" in attr_name:
            # Create degree Celsius symbol for temperature
            setattr(dashboard, f"{attr_name}_unit", tk.Label(value_frame, text="°C"))
            getattr(dashboard, f"{attr_name}_unit").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_unit").pack_forget()  # Hide initially
        elif "bandwidth" in attr_name:
            # Create Kb/s symbol for bandwidth
            setattr(dashboard, f"{attr_name}_unit", tk.Label(value_frame, text="Kb/s"))
            getattr(dashboard, f"{attr_name}_unit").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_unit").pack_forget()  # Hide initially
        else:
            # Create percentage symbol for other parameters
            setattr(dashboard, f"{attr_name}_pct", tk.Label(value_frame, text="%"))
            getattr(dashboard, f"{attr_name}_pct").pack(side=tk.LEFT, padx=0)
            getattr(dashboard, f"{attr_name}_pct").pack_forget()  # Hide initially
        
        dashboard.robot2_params[attr_name] = "N/A"
    
    # Configure robot frames to expand
    dashboard.robots_frame.grid_columnconfigure(0, weight=1)
    dashboard.robots_frame.grid_columnconfigure(1, weight=1)

def create_rpm_tab(dashboard):
    """Create the RPM tab with progress bars for motor RPM values"""
    # Create frame for two robot RPM status
    dashboard.rpm_frame = tk.Frame(dashboard.rpm_tab)
    dashboard.rpm_frame.pack(fill=tk.BOTH, expand=True)
    
    # Robot 1 RPM
    dashboard.robot1_rpm_frame = tk.LabelFrame(dashboard.rpm_frame, text="ROBOT 1", padx=5, pady=2)
    dashboard.robot1_rpm_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=2)
    
    # Robot 1 RPM parameters
    dashboard.robot1_rpm_params = {}
    rpm_params = [
        ("Top Motor", "rpm_top_1"),
        ("Bottom Motor", "rpm_bottom_1")
    ]
    
    # Create RPM indicators for Robot 1
    for idx, (label_text, attr_name) in enumerate(rpm_params):
        # Row for the label
        label_row = idx * 3
        # Row for the value display
        value_row = label_row + 1
        # Row for the progress bar
        bar_row = value_row + 1
        
        # Label for motor name - move to center and make it span both columns
        tk.Label(dashboard.robot1_rpm_frame, text=label_text).grid(
            row=label_row, column=0, columnspan=2, sticky="", pady=(10,0))
        
        # Create a frame for the RPM value display
        value_frame = tk.Frame(dashboard.robot1_rpm_frame)
        value_frame.grid(row=value_row, column=0, columnspan=2, sticky="", pady=(0,0))
        
        # RPM value display - modify to use larger font
        setattr(dashboard, attr_name, tk.Label(value_frame, text="0", 
                                              font=("Arial", 24, "bold")))
        getattr(dashboard, attr_name).pack(side=tk.LEFT)
        
        # RPM unit - place next to the value with smaller font
        rpm_unit = tk.Label(value_frame, text="RPM", font=("Arial", 10))
        rpm_unit.pack(side=tk.LEFT, padx=(5,0), pady=(10,0))
        
        # Progress bar for RPM
        setattr(dashboard, f"{attr_name}_bar", ttk.Progressbar(
            dashboard.robot1_rpm_frame, 
            orient="horizontal", 
            length=200, 
            mode="determinate",
            maximum=MAX_RPM
        ))
        getattr(dashboard, f"{attr_name}_bar").grid(row=bar_row, column=0, columnspan=2, sticky="ew", padx=5, pady=(5,10))
        
        # Initialize RPM values
        dashboard.robot1_rpm_params[attr_name] = 0
    
    # Robot 2 RPM
    dashboard.robot2_rpm_frame = tk.LabelFrame(dashboard.rpm_frame, text="ROBOT 2", padx=5, pady=2)
    dashboard.robot2_rpm_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=2)
    
    # Robot 2 RPM parameters
    dashboard.robot2_rpm_params = {}
    rpm_params = [
        ("Top Motor", "rpm_top_2"),
        ("Bottom Motor", "rpm_bottom_2")
    ]
    
    # Create RPM indicators for Robot 2
    for idx, (label_text, attr_name) in enumerate(rpm_params):
        # Row for the label
        label_row = idx * 3
        # Row for the value display
        value_row = label_row + 1
        # Row for the progress bar
        bar_row = value_row + 1
        
        # Label for motor name - move to center and make it span both columns
        tk.Label(dashboard.robot2_rpm_frame, text=label_text).grid(
            row=label_row, column=0, columnspan=2, sticky="", pady=(10,0))
        
        # Create a frame for the RPM value display
        value_frame = tk.Frame(dashboard.robot2_rpm_frame)
        value_frame.grid(row=value_row, column=0, columnspan=2, sticky="", pady=(0,0))
        
        # RPM value display - modify to use larger font
        setattr(dashboard, attr_name, tk.Label(value_frame, text="0", 
                                              font=("Arial", 24, "bold")))
        getattr(dashboard, attr_name).pack(side=tk.LEFT)
        
        # RPM unit - place next to the value with smaller font
        rpm_unit = tk.Label(value_frame, text="RPM", font=("Arial", 10))
        rpm_unit.pack(side=tk.LEFT, padx=(5,0), pady=(10,0))
        
        # Progress bar for RPM
        setattr(dashboard, f"{attr_name}_bar", ttk.Progressbar(
            dashboard.robot2_rpm_frame, 
            orient="horizontal", 
            length=200, 
            mode="determinate",
            maximum=MAX_RPM
        ))
        getattr(dashboard, f"{attr_name}_bar").grid(row=bar_row, column=0, columnspan=2, sticky="ew", padx=5, pady=(5,10))
        
        # Initialize RPM values
        dashboard.robot2_rpm_params[attr_name] = 0
    
    # Configure RPM frames to expand
    dashboard.rpm_frame.grid_columnconfigure(0, weight=1)
    dashboard.rpm_frame.grid_columnconfigure(1, weight=1)
    
    # Setup RPM subscriptions
    for i in range(1, 3):
        rpm_topic = f"/ROBOT{i}/shot"
        if rpm_topic in dashboard.available_topics:
            subscribe_to_rpm(dashboard, rpm_topic, i)

def update_rpm_progress_bar(dashboard, robot_num, motor_type, rpm_value):
    """Update the RPM progress bar with appropriate color based on value"""
    # Convert to integer and ensure within range
    rpm = min(int(float(rpm_value)), MAX_RPM)
    
    # Get the attribute names
    attr_name = f"rpm_{motor_type}_{robot_num}"
    bar_attr_name = f"{attr_name}_bar"
    
    # Update the value label
    if hasattr(dashboard, attr_name):
        getattr(dashboard, attr_name).config(text=str(rpm))
    
    # Update the progress bar value
    if hasattr(dashboard, bar_attr_name):
        bar = getattr(dashboard, bar_attr_name)
        bar["value"] = rpm
        
        # Determine color based on RPM value
        if rpm < RPM_WARNING_THRESHOLD:
            # Green for normal operation
            bar.config(style="Green.Horizontal.TProgressbar")
        elif rpm < RPM_CRITICAL_THRESHOLD:
            # Orange for warning
            bar.config(style="Orange.Horizontal.TProgressbar")
        else:
            # Red for critical
            bar.config(style="Red.Horizontal.TProgressbar")

def subscribe_to_rpm(dashboard, topic_name, robot_num):
    """Subscribe to RPM topic for the specified robot"""
    # Unsubscribe if already subscribed
    rpm_sub_attr = f"rpm_topic{robot_num}_sub"
    if hasattr(dashboard, rpm_sub_attr):
        dashboard.node.destroy_subscription(getattr(dashboard, rpm_sub_attr))
    
    # Create custom style for progress bars
    style = ttk.Style()
    style.configure("Green.Horizontal.TProgressbar", background='green')
    style.configure("Orange.Horizontal.TProgressbar", background='orange')
    style.configure("Red.Horizontal.TProgressbar", background='red')
    
    # Set default style for all progress bars
    for robot in [1, 2]:
        for motor in ["top", "bottom"]:
            bar_attr = f"rpm_{motor}_{robot}_bar"
            if hasattr(dashboard, bar_attr):
                getattr(dashboard, bar_attr).config(style="Green.Horizontal.TProgressbar")
    
    # Callback function for RPM data
    def rpm_callback(msg):
        try:
            # Format: "RPM {top_motor} {bottom_motor}"
            data = str(msg.data).split()
            if len(data) >= 4 and data[0].upper() == "SA":
                top_rpm = data[2]
                bottom_rpm = data[3]
                
                # Update RPM values and progress bars
                update_rpm_progress_bar(dashboard, robot_num, "top", top_rpm)
                update_rpm_progress_bar(dashboard, robot_num, "bottom", bottom_rpm)
                
        except Exception as e:
            print(f"Error parsing RPM message for Robot {robot_num}: {e}")
    
    # Subscribe to the RPM topic
    try:
        from std_msgs.msg import String
        sub = dashboard.node.create_subscription(
            String,
            topic_name,
            rpm_callback,
            10
        )
        setattr(dashboard, rpm_sub_attr, sub)
        print(f"Subscribed to {topic_name} for Robot {robot_num} RPM")
    except Exception as e:
        print(f"Error subscribing to RPM topic {topic_name} for Robot {robot_num}: {e}")