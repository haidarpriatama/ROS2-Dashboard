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
    
    # Robot selection (radio buttons)
    robot_frame = tk.Frame(dashboard.pid_tab)
    robot_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
    
    # Create IntVar for robot selection (1 or 2)
    dashboard.pid_robot_var = tk.IntVar(value=1)  # Default to Robot 1
    
    # Radio buttons for Robot 1 and Robot 2
    tk.Label(robot_frame, text="Robot:").pack(side=tk.LEFT, padx=5)
    tk.Radiobutton(robot_frame, text="Robot 1", variable=dashboard.pid_robot_var, value=1).pack(side=tk.LEFT, padx=5)
    tk.Radiobutton(robot_frame, text="Robot 2", variable=dashboard.pid_robot_var, value=2).pack(side=tk.LEFT, padx=5)
    
    # PID input fields
    pid_inputs_frame = tk.Frame(dashboard.pid_tab)
    pid_inputs_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
    
    # P value
    p_frame = tk.Frame(pid_inputs_frame)
    p_frame.pack(fill=tk.X, pady=2)
    tk.Label(p_frame, text="P value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_p_entry = tk.Entry(p_frame)
    dashboard.pid_p_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # I value
    i_frame = tk.Frame(pid_inputs_frame)
    i_frame.pack(fill=tk.X, pady=2)
    tk.Label(i_frame, text="I value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_i_entry = tk.Entry(i_frame)
    dashboard.pid_i_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # D value
    d_frame = tk.Frame(pid_inputs_frame)
    d_frame.pack(fill=tk.X, pady=2)
    tk.Label(d_frame, text="D value:", width=8, anchor="e").pack(side=tk.LEFT, padx=5)
    dashboard.pid_d_entry = tk.Entry(d_frame)
    dashboard.pid_d_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
    # Load PID values if they exist in configuration
    if hasattr(dashboard, 'pid_p_value'):
        dashboard.pid_p_entry.insert(0, dashboard.pid_p_value)
    if hasattr(dashboard, 'pid_i_value'):
        dashboard.pid_i_entry.insert(0, dashboard.pid_i_value)
    if hasattr(dashboard, 'pid_d_value'):
        dashboard.pid_d_entry.insert(0, dashboard.pid_d_value)
    if hasattr(dashboard, 'pid_robot'):
        dashboard.pid_robot_var.set(dashboard.pid_robot)
    
    # Send PID button
    dashboard.pid_send_button = tk.Button(
        dashboard.pid_tab, 
        text="SEND PID", 
        command=lambda: send_pid_values(dashboard)
    )
    dashboard.pid_send_button.grid(
        row=2, column=0, pady=5
    )
    
    # PID Feedback label
    dashboard.pid_feedback_label = tk.Label(
        dashboard.pid_tab, 
        text="", 
        wraplength=250, 
        fg="dark green"
    )
    dashboard.pid_feedback_label.grid(
        row=3, column=0, pady=(0,2)
    )

def update_output_topic(dashboard):
    """Update the output topic when selected from the combobox"""
    topic = dashboard.output_topics.get()
    if topic:
        dashboard.output_topic = topic
        # Save configuration immediately when output topic changes
        dashboard.save_config()

def send_pid_values(dashboard):
    """Send PID values to the selected robot's serialTX topic"""
    # Get selected robot number
    robot_num = dashboard.pid_robot_var.get()
    
    # Get PID values
    try:
        p_value = float(dashboard.pid_p_entry.get())
        i_value = float(dashboard.pid_i_entry.get())
        d_value = float(dashboard.pid_d_entry.get())
    except ValueError:
        # Show error message if values are not valid numbers
        dashboard.pid_feedback_label.config(
            text="Error: P, I, and D must be valid numbers",
            fg="red"
        )
        return
    
    # Construct the message with the correct format
    message = f"PID x {p_value} {i_value} {d_value}"
    
    # Topic to send to
    topic = f"/ROBOT{robot_num}/serialTX"
    
    # Save the PID values in dashboard for configuration saving
    dashboard.pid_p_value = p_value
    dashboard.pid_i_value = i_value
    dashboard.pid_d_value = d_value
    dashboard.pid_robot = robot_num
    
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
    dashboard.pid_feedback_label.config(
        text=f"Sent PID values to {topic}:\n{message}",
        fg="dark green"
    )
    
    # Save configuration
    dashboard.save_config()