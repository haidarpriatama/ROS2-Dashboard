import tkinter as tk
from tkinter import ttk

def create_send_data_widget(dashboard):
    """Create the send data widget"""
    # Send Data widget
    dashboard.send_data_frame = tk.LabelFrame(dashboard.main_frame, text="SEND DATA", padx=5, pady=5)
    dashboard.send_data_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
    
    # Configure grid to make frame expandable
    dashboard.send_data_frame.grid_columnconfigure(0, weight=1)
    
    # Create a notebook for tabs
    dashboard.send_data_notebook = ttk.Notebook(dashboard.send_data_frame)
    dashboard.send_data_notebook.pack(fill=tk.BOTH, expand=True)
    
    # Create the "Inject" tab (original functionality)
    dashboard.inject_tab = ttk.Frame(dashboard.send_data_notebook)
    dashboard.send_data_notebook.add(dashboard.inject_tab, text="Inject")
    
    # Create the "PID" tab (new functionality)
    dashboard.pid_tab = ttk.Frame(dashboard.send_data_notebook)
    dashboard.send_data_notebook.add(dashboard.pid_tab, text="PID")
    
    # Configure grid for inject tab
    dashboard.inject_tab.grid_columnconfigure(0, weight=1)
    
    # ---- INJECT TAB ----
    # Original send data functionality
    tk.Label(dashboard.inject_tab, text="APP \u2192 /Topic").grid(
        row=0, column=0, sticky="w", pady=(0,2), padx=5
    )
    dashboard.input_entry = tk.Entry(dashboard.inject_tab)
    dashboard.input_entry.grid(
        row=1, column=0, sticky="ew", padx=5, pady=(0,5)
    )
    
    # Bind Enter key to send_data function
    dashboard.input_entry.bind("<Return>", lambda event: dashboard.send_data())
    
    # Topic output selection
    tk.Label(dashboard.inject_tab, text="/Topic output").grid(
        row=2, column=0, sticky="w", pady=(0,2), padx=5
    )
    dashboard.output_topics = ttk.Combobox(dashboard.inject_tab)
    dashboard.output_topics.grid(
        row=3, column=0, sticky="ew", padx=5, pady=(0,5)
    )
    
    # Set output topic if already configured
    if hasattr(dashboard, 'output_topic') and dashboard.output_topic:
        dashboard.output_topics.set(dashboard.output_topic)
    
    # Bind the output topic selection to save configuration
    dashboard.output_topics.bind("<<ComboboxSelected>>", 
                            lambda e: update_output_topic(dashboard))
    
    # Send button
    dashboard.send_button = tk.Button(
        dashboard.inject_tab, 
        text="SEND", 
        command=dashboard.send_data
    )
    dashboard.send_button.grid(
        row=4, column=0, pady=5
    )
    
    # Feedback label
    dashboard.feedback_label = tk.Label(
        dashboard.inject_tab, 
        text="", 
        wraplength=250, 
        fg="dark green"
    )
    dashboard.feedback_label.grid(
        row=5, column=0, pady=(0,2)
    )
    
    # ---- PID TAB ----
    # Configure grid for PID tab
    dashboard.pid_tab.grid_columnconfigure(0, weight=1)
    
    # Create a nested notebook for Robot 1 and Robot 2 PID settings
    dashboard.pid_robot_notebook = ttk.Notebook(dashboard.pid_tab)
    dashboard.pid_robot_notebook.pack(fill=tk.BOTH, expand=True)
    
    # Create tabs for Robot 1 and Robot 2
    dashboard.pid_robot1_tab = ttk.Frame(dashboard.pid_robot_notebook)
    dashboard.pid_robot_notebook.add(dashboard.pid_robot1_tab, text="Robot 1")
    
    dashboard.pid_robot2_tab = ttk.Frame(dashboard.pid_robot_notebook)
    dashboard.pid_robot_notebook.add(dashboard.pid_robot2_tab, text="Robot 2")
    
    # Configure grid for both robot tabs
    dashboard.pid_robot1_tab.grid_columnconfigure(0, weight=1)
    dashboard.pid_robot2_tab.grid_columnconfigure(0, weight=1)
    
    # ----- ROBOT 1 PID INPUTS -----
    # PID input fields for Robot 1
    pid1_inputs_frame = tk.Frame(dashboard.pid_robot1_tab)
    pid1_inputs_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
    
    # P value for Robot 1
    p1_frame = tk.Frame(pid1_inputs_frame)
    p1_frame.pack(fill=tk.X, pady=2)
    tk.Label(p1_frame, text="P value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_p1_entry = tk.Entry(p1_frame)
    dashboard.pid_p1_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # I value for Robot 1
    i1_frame = tk.Frame(pid1_inputs_frame)
    i1_frame.pack(fill=tk.X, pady=2)
    tk.Label(i1_frame, text="I value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_i1_entry = tk.Entry(i1_frame)
    dashboard.pid_i1_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # D value for Robot 1
    d1_frame = tk.Frame(pid1_inputs_frame)
    d1_frame.pack(fill=tk.X, pady=2)
    tk.Label(d1_frame, text="D value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_d1_entry = tk.Entry(d1_frame)
    dashboard.pid_d1_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Send PID button for Robot 1
    dashboard.pid_send1_button = tk.Button(
        dashboard.pid_robot1_tab, 
        text="SEND PID TO ROBOT 1", 
        command=lambda: send_pid_values(dashboard, 1)
    )
    dashboard.pid_send1_button.grid(
        row=1, column=0, pady=5
    )
    
    # PID Feedback label for Robot 1
    dashboard.pid_feedback1_label = tk.Label(
        dashboard.pid_robot1_tab, 
        text="", 
        wraplength=500, 
        fg="dark green"
    )
    dashboard.pid_feedback1_label.grid(
        row=2, column=0, pady=(0,2)
    )
    
    # ----- ROBOT 2 PID INPUTS -----
    # PID input fields for Robot 2
    pid2_inputs_frame = tk.Frame(dashboard.pid_robot2_tab)
    pid2_inputs_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
    
    # P value for Robot 2
    p2_frame = tk.Frame(pid2_inputs_frame)
    p2_frame.pack(fill=tk.X, pady=2)
    tk.Label(p2_frame, text="P value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_p2_entry = tk.Entry(p2_frame)
    dashboard.pid_p2_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # I value for Robot 2
    i2_frame = tk.Frame(pid2_inputs_frame)
    i2_frame.pack(fill=tk.X, pady=2)
    tk.Label(i2_frame, text="I value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_i2_entry = tk.Entry(i2_frame)
    dashboard.pid_i2_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # D value for Robot 2
    d2_frame = tk.Frame(pid2_inputs_frame)
    d2_frame.pack(fill=tk.X, pady=2)
    tk.Label(d2_frame, text="D value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_d2_entry = tk.Entry(d2_frame)
    dashboard.pid_d2_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Send PID button for Robot 2
    dashboard.pid_send2_button = tk.Button(
        dashboard.pid_robot2_tab, 
        text="SEND PID TO ROBOT 2", 
        command=lambda: send_pid_values(dashboard, 2)
    )
    dashboard.pid_send2_button.grid(
        row=1, column=0, pady=5
    )
    
    # PID Feedback label for Robot 2
    dashboard.pid_feedback2_label = tk.Label(
        dashboard.pid_robot2_tab, 
        text="", 
        wraplength=500, 
        fg="dark green"
    )
    dashboard.pid_feedback2_label.grid(
        row=2, column=0, pady=(0,2)
    )
    
    # Load PID values if they exist in configuration
    # For Robot 1
    if hasattr(dashboard, 'pid_p1_value'):
        dashboard.pid_p1_entry.insert(0, dashboard.pid_p1_value)
    if hasattr(dashboard, 'pid_i1_value'):
        dashboard.pid_i1_entry.insert(0, dashboard.pid_i1_value)
    if hasattr(dashboard, 'pid_d1_value'):
        dashboard.pid_d1_entry.insert(0, dashboard.pid_d1_value)
    
    # For Robot 2
    if hasattr(dashboard, 'pid_p2_value'):
        dashboard.pid_p2_entry.insert(0, dashboard.pid_p2_value)
    if hasattr(dashboard, 'pid_i2_value'):
        dashboard.pid_i2_entry.insert(0, dashboard.pid_i2_value)
    if hasattr(dashboard, 'pid_d2_value'):
        dashboard.pid_d2_entry.insert(0, dashboard.pid_d2_value)

def update_output_topic(dashboard):
    """Update the output topic when selected from the combobox"""
    topic = dashboard.output_topics.get()
    if topic:
        dashboard.output_topic = topic
        # Save configuration immediately when output topic changes
        dashboard.save_config()

def send_pid_values(dashboard, robot_num):
    """Send PID values to the specified robot's serialTX topic"""
    # Determine which robot's PID values to send
    p_entry = getattr(dashboard, f"pid_p{robot_num}_entry")
    i_entry = getattr(dashboard, f"pid_i{robot_num}_entry")
    d_entry = getattr(dashboard, f"pid_d{robot_num}_entry")
    feedback_label = getattr(dashboard, f"pid_feedback{robot_num}_label")
    
    # Get PID values
    try:
        p_value = float(p_entry.get())
        i_value = float(i_entry.get())
        d_value = float(d_entry.get())
    except ValueError:
        # Show error message if values are not valid numbers
        feedback_label.config(
            text="Error: P, I, and D must be valid numbers",
            fg="red"
        )
        return
    
    # Construct the message with the correct format
    message = f"PID x {p_value} {i_value} {d_value}"
    
    # Topic to send to
    topic = f"/ROBOT{robot_num}/serialTX"
    
    # Save the PID values in dashboard for configuration saving
    setattr(dashboard, f"pid_p{robot_num}_value", p_value)
    setattr(dashboard, f"pid_i{robot_num}_value", i_value)
    setattr(dashboard, f"pid_d{robot_num}_value", d_value)
    
    from std_msgs.msg import String
    
    # Create publisher if not exists
    pub_attr = f"pid_publisher_{robot_num}"
    if not hasattr(dashboard, pub_attr):
        setattr(dashboard, pub_attr, dashboard.node.create_publisher(
            String,
            topic,
            10
        ))
    
    # Create and publish message
    msg = String()
    msg.data = message
    getattr(dashboard, pub_attr).publish(msg)
    
    # Update feedback label
    feedback_label.config(
        text=f"Sent PID values to {topic} : {message}",
        fg="dark green"
    )
    
    # Save configuration
    dashboard.save_config()