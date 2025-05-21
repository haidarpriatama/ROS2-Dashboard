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
    
    # Create selection frame for robot and axis
    selection_frame = tk.Frame(dashboard.pid_tab)
    selection_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
    
    # Robot selection (Robot 1 or Robot 2)
    robot_frame = tk.LabelFrame(selection_frame, text="Robot")
    robot_frame.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
    
    dashboard.selected_robot = tk.IntVar(value=1)  # Default to Robot 1
    
    # Robot 1 radio button
    robot1_radio = tk.Radiobutton(
        robot_frame, 
        text="Robot 1", 
        variable=dashboard.selected_robot, 
        value=1,
        command=lambda: update_pid_display(dashboard)
    )
    robot1_radio.pack(side=tk.LEFT, padx=10)
    
    # Robot 2 radio button
    robot2_radio = tk.Radiobutton(
        robot_frame, 
        text="Robot 2", 
        variable=dashboard.selected_robot, 
        value=2,
        command=lambda: update_pid_display(dashboard)
    )
    robot2_radio.pack(side=tk.LEFT, padx=10)
    
    # Axis selection (x, y, or yaw)
    axis_frame = tk.LabelFrame(selection_frame, text="Axis")
    axis_frame.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
    
    dashboard.selected_axis = tk.StringVar(value="x")  # Default to x-axis
    
    # X-axis radio button
    x_radio = tk.Radiobutton(
        axis_frame, 
        text="X", 
        variable=dashboard.selected_axis, 
        value="x",
        command=lambda: update_pid_display(dashboard)
    )
    x_radio.pack(side=tk.LEFT, padx=5)
    
    # Y-axis radio button
    y_radio = tk.Radiobutton(
        axis_frame, 
        text="Y", 
        variable=dashboard.selected_axis, 
        value="y",
        command=lambda: update_pid_display(dashboard)
    )
    y_radio.pack(side=tk.LEFT, padx=5)
    
    # Yaw-axis radio button
    yaw_radio = tk.Radiobutton(
        axis_frame, 
        text="Yaw", 
        variable=dashboard.selected_axis, 
        value="yaw",
        command=lambda: update_pid_display(dashboard)
    )
    yaw_radio.pack(side=tk.LEFT, padx=5)
    
    # Main PID frame (shared between robots and axes)
    dashboard.pid_main_frame = tk.Frame(dashboard.pid_tab)
    dashboard.pid_main_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
    
    # Frame for PID inputs
    pid_inputs_frame = tk.Frame(dashboard.pid_main_frame)
    pid_inputs_frame.pack(fill=tk.X, pady=2)
    
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
    
    # Send PID button (text will be updated based on selected robot and axis)
    dashboard.pid_send_button = tk.Button(
        dashboard.pid_main_frame, 
        text="SEND PID TO ROBOT 1 (X)",
        command=lambda: send_pid_values(dashboard)
    )
    dashboard.pid_send_button.pack(pady=5)
    
    # PID Feedback label
    dashboard.pid_feedback_label = tk.Label(
        dashboard.pid_main_frame, 
        text="", 
        wraplength=500, 
        fg="dark green"
    )
    dashboard.pid_feedback_label.pack(pady=(0,2))
    
    # Initialize with Robot 1, X-axis PID values
    update_pid_display(dashboard)

def update_output_topic(dashboard):
    """Update the output topic when selected from the combobox"""
    topic = dashboard.output_topics.get()
    if topic:
        dashboard.output_topic = topic
        # Save configuration immediately when output topic changes
        dashboard.save_config()

def update_pid_display(dashboard):
    """Update PID input fields based on selected robot and axis"""
    robot_num = dashboard.selected_robot.get()
    axis = dashboard.selected_axis.get()
    
    # Update send button text
    dashboard.pid_send_button.config(text=f"SEND PID TO ROBOT {robot_num} ({axis.upper()})")
    
    # Clear entries
    dashboard.pid_p_entry.delete(0, tk.END)
    dashboard.pid_i_entry.delete(0, tk.END)
    dashboard.pid_d_entry.delete(0, tk.END)
    
    # Generate attribute names based on robot number and axis
    p_attr = f"pid_p{robot_num}_{axis}_value"
    i_attr = f"pid_i{robot_num}_{axis}_value"
    d_attr = f"pid_d{robot_num}_{axis}_value"
    
    # Load values if they exist
    if hasattr(dashboard, p_attr):
        dashboard.pid_p_entry.insert(0, str(getattr(dashboard, p_attr)))
    if hasattr(dashboard, i_attr):
        dashboard.pid_i_entry.insert(0, str(getattr(dashboard, i_attr)))
    if hasattr(dashboard, d_attr):
        dashboard.pid_d_entry.insert(0, str(getattr(dashboard, d_attr)))
    
    # Clear feedback
    dashboard.pid_feedback_label.config(text="")

def send_pid_values(dashboard):
    """Send PID values to the specified robot's serialTX topic with selected axis"""
    # Get selected robot and axis
    robot_num = dashboard.selected_robot.get()
    axis = dashboard.selected_axis.get()
    
    # Get current timestamp
    import datetime
    current_time = datetime.datetime.now().strftime("%H:%M:%S %d/%m/%Y")
    display_time = datetime.datetime.now().strftime("%H:%M:%S")
    
    # Get PID values from the shared entry fields
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
    message = f"PID {axis} {p_value} {i_value} {d_value}"
    
    # Topic to send to
    topic = f"/ROBOT{robot_num}/serialTX"
    
    # Generate attribute names based on robot number and axis
    p_attr = f"pid_p{robot_num}_{axis}_value"
    i_attr = f"pid_i{robot_num}_{axis}_value"
    d_attr = f"pid_d{robot_num}_{axis}_value"
    
    # Save the PID values in dashboard for configuration saving
    setattr(dashboard, p_attr, p_value)
    setattr(dashboard, i_attr, i_value)
    setattr(dashboard, d_attr, d_value)
    
    # Set the PID timestamp for config
    setattr(dashboard, "pid_last_update", current_time)
    
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
    
    # Update feedback label with timestamp
    dashboard.pid_feedback_label.config(
        text=f"[{display_time}] Sent PID to Robot {robot_num} ({axis.upper()}): P={p_value}, I={i_value}, D={d_value}",
        fg="dark green"
    )
    
    # Print to console for logging
    print(f"[{current_time}] Sent PID values to Robot {robot_num} ({axis.upper()}): P={p_value}, I={i_value}, D={d_value}")
    
    # Save configuration
    dashboard.save_config()