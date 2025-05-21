import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

def create_visual_widget(dashboard):
    """Create the visual (position and graph) widget"""
    # Position widget
    dashboard.position_frame = tk.LabelFrame(dashboard.main_frame, text="VISUAL TABS", padx=5, pady=5)
    dashboard.position_frame.grid(row=1, column=1, columnspan=2, sticky="nsew", padx=(0, 0), pady=5)
    
    # Create a notebook (tabbed interface)
    dashboard.position_notebook = ttk.Notebook(dashboard.position_frame)
    dashboard.position_notebook.pack(fill=tk.BOTH, expand=True)
    
    # Create Map tab
    dashboard.map_tab = ttk.Frame(dashboard.position_notebook)
    dashboard.position_notebook.add(dashboard.map_tab, text="Map")
    
    # Create Graph tab
    dashboard.graph_tab = ttk.Frame(dashboard.position_notebook)
    dashboard.position_notebook.add(dashboard.graph_tab, text="Graph")
    
    # ---- MAP TAB ----
    # Top frame to hold both topic selection and team toggle button
    top_frame = tk.Frame(dashboard.map_tab)
    top_frame.pack(fill=tk.X, pady=2)
    
    # Topic selection for robots' positions (left side)
    map_topics_frame = tk.Frame(top_frame)
    map_topics_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
    
    # Team toggle button frame (right side)
    team_toggle_frame = tk.Frame(top_frame)
    team_toggle_frame.pack(side=tk.RIGHT, padx=10)
    
    # Create team toggle button
    dashboard.current_team = 1  # Default to Team 1 view
    dashboard.team_toggle_btn = tk.Button(
        team_toggle_frame, 
        text="Current: Team 1",
        fg="#FF0000",  # Warna merah untuk Team 1
        width=15,
        command=lambda: toggle_team_view(dashboard)
    )
    dashboard.team_toggle_btn.pack()
    
    # Robot 1 position topic selection
    tk.Label(map_topics_frame, text="Robot 1:").grid(row=0, column=0, sticky="w", padx=2)
    dashboard.pos_topic1_combo = ttk.Combobox(map_topics_frame, values=dashboard.available_topics, width=30)  
    dashboard.pos_topic1_combo.grid(row=0, column=1, padx=2, pady=2)
    # Add binding for Robot 1 position topic selection
    dashboard.pos_topic1_combo.bind("<<ComboboxSelected>>", 
                     lambda e: dashboard.subscribe_to_position(dashboard.pos_topic1_combo.get(), 1))

    # Robot 1 color box
    robot1_color_box = tk.Label(map_topics_frame, bg="#FF0000", width=2, height=1, relief=tk.SOLID)  # Merah untuk Robot 1
    robot1_color_box.grid(row=0, column=2, padx=5)

    # Robot 2 position topic selection
    tk.Label(map_topics_frame, text="Robot 2:").grid(row=0, column=3, sticky="w", padx=2)
    dashboard.pos_topic2_combo = ttk.Combobox(map_topics_frame, values=dashboard.available_topics, width=30) 
    dashboard.pos_topic2_combo.grid(row=0, column=4, padx=2, pady=2)
    # Add binding for Robot 2 position topic selection
    dashboard.pos_topic2_combo.bind("<<ComboboxSelected>>", 
                     lambda e: dashboard.subscribe_to_position(dashboard.pos_topic2_combo.get(), 2))

    # Robot 2 color box
    robot2_color_box = tk.Label(map_topics_frame, bg="#0000FF", width=2, height=1, relief=tk.SOLID)  # Biru untuk Robot 2
    robot2_color_box.grid(row=0, column=5, padx=5)
    
    # Coordinates display
    coords_frame = tk.Frame(dashboard.map_tab)
    coords_frame.pack(fill=tk.X, pady=2)
    
    # Robot 1 coordinates
    tk.Label(coords_frame, text="Robot 1 (X, Y):").grid(row=0, column=0, sticky="w", padx=2)
    dashboard.robot1_coords = tk.Label(coords_frame, text="(0, 0)")
    dashboard.robot1_coords.grid(row=0, column=1, sticky="w", padx=2)
    
    # Robot 2 coordinates
    tk.Label(coords_frame, text="Robot 2 (X, Y):").grid(row=0, column=2, sticky="w", padx=10)
    dashboard.robot2_coords = tk.Label(coords_frame, text="(0, 0)")
    dashboard.robot2_coords.grid(row=0, column=3, sticky="w", padx=2)
    
    # Create matplotlib figure for map
    dashboard.fig = Figure(figsize=(7, 4), dpi=100)  # Adjust figure size
    dashboard.ax = dashboard.fig.add_subplot(111)
    
    # Load field image
    try:
        dashboard.map_image_path = dashboard.resources.MAP_IMAGE
        
        # Load the image
        img = plt.imread(dashboard.map_image_path)
        dashboard.field_img = dashboard.ax.imshow(img, extent=[1500, 0, 0, 800])  # Set map scale to 1500 x 800
        
        # Store the latest robot positions (for toggling between team views)
        dashboard.robot1_pos_data = [0, 0]  # [x, y]
        dashboard.robot2_pos_data = [0, 0]  # [x, y]
        
        # Set custom x-axis ticks to ensure 1500 is visible
        x_ticks = [0, 250, 500, 750, 1000, 1250, 1500]
        dashboard.ax.set_xticks(x_ticks)
        
        # Grid and other settings
        dashboard.ax.grid(True)
        
        # Move y-axis label to right side
        dashboard.ax.yaxis.set_label_position("right")
        dashboard.ax.yaxis.tick_right()

        # Initialize robot positions (start at origin)
        dashboard.robot2_pos = dashboard.ax.plot(0, 0, marker='o', mfc='#0000FF', mec='#1C045EFF', mew=2, markersize=40)[0]  # Biru muda untuk Robot 2
        dashboard.robot1_pos = dashboard.ax.plot(0, 0, marker='o', mfc='#FF0000', mec='#6E0000FF', mew=2, markersize=40)[0]  # Oranye untuk Robot 1

        # Create canvas
        dashboard.canvas = FigureCanvasTkAgg(dashboard.fig, dashboard.map_tab)
        dashboard.canvas.draw()
        dashboard.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    except Exception as e:
        print(f"Error loading field map: {e}")
        tk.Label(dashboard.map_tab, text="Error loading map").pack()
    
    # ---- GRAPH TAB ----
    # Create main graph frame that will contain all elements
    dashboard.graph_main_frame = tk.Frame(dashboard.graph_tab)
    dashboard.graph_main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Graph topic selection frame (at the top)
    graph_topics_frame = tk.Frame(dashboard.graph_main_frame)
    graph_topics_frame.pack(fill=tk.X, pady=2)
    
    # Graph topic selection
    tk.Label(graph_topics_frame, text="Graph Topic:").grid(row=0, column=0, sticky="w", padx=2)
    dashboard.graph_topic_combo = ttk.Combobox(graph_topics_frame, values=dashboard.available_topics, width=50)
    dashboard.graph_topic_combo.grid(row=0, column=1, padx=2, pady=2)
    dashboard.graph_topic_combo.bind("<<ComboboxSelected>>", 
                        lambda e: dashboard.subscribe_to_graph_topic(dashboard.graph_topic_combo.get()))
    
    # Add graph control (Play/Pause) buttons
    dashboard.graph_paused = False  # Flag to track if graph updating is paused
    dashboard.pause_button = ttk.Button(graph_topics_frame, text="Pause", width=10,
                                       command=lambda: toggle_graph_pause(dashboard))
    dashboard.pause_button.grid(row=0, column=2, padx=5, pady=2)
    
    dashboard.reset_button = ttk.Button(graph_topics_frame, text="Reset Time", width=10,
                                       command=lambda: reset_graph_time(dashboard))
    dashboard.reset_button.grid(row=0, column=3, padx=5, pady=2)
    
    # Add status display at the top (above the graph) 
    dashboard.graph_status_frame = tk.Frame(dashboard.graph_main_frame)
    dashboard.graph_status_frame.pack(fill=tk.X, pady=2)
    
    dashboard.graph_status_label = tk.Label(dashboard.graph_status_frame, text="Status: Running", 
                                           font=("Arial", 9), bd=1, relief=tk.SUNKEN, anchor=tk.W, padx=5)
    dashboard.graph_status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
    
    dashboard.graph_time_label = tk.Label(dashboard.graph_status_frame, text="Elapsed: 0.00s", 
                                        font=("Arial", 9), bd=1, relief=tk.SUNKEN, anchor=tk.E, padx=5)
    dashboard.graph_time_label.pack(side=tk.RIGHT, fill=tk.X, expand=True)
    
    # Add auto-center option
    dashboard.auto_center_var = tk.BooleanVar(value=True)
    dashboard.auto_center_check = tk.Checkbutton(dashboard.graph_status_frame, 
                                                text="Auto-center setpoints", 
                                                variable=dashboard.auto_center_var)
    dashboard.auto_center_check.pack(side=tk.RIGHT, padx=10)
    
    # Create graph frame to contain the matplotlib canvas
    dashboard.graph_plot_frame = tk.Frame(dashboard.graph_main_frame, width=500)
    dashboard.graph_plot_frame.pack(fill=tk.BOTH, expand=True, pady=2, padx=(10, 10))   # Add horizontal padding
    dashboard.graph_plot_frame.pack_propagate(False)  # Prevent resizing based on contents
    
    # Create matplotlib figure for graph
    dashboard.graph_fig = Figure(figsize=(3, 4), dpi=100)
    dashboard.graph_ax1 = dashboard.graph_fig.add_subplot(2, 1, 1)
    dashboard.graph_ax2 = dashboard.graph_fig.add_subplot(2, 1, 2)
    
    # Initialize graph lines
    dashboard.graph_line_x, = dashboard.graph_ax1.plot([], [], 'b-', label='X')
    dashboard.graph_line_spx, = dashboard.graph_ax1.plot([], [], 'k-', label='SPX')
    dashboard.graph_line_y, = dashboard.graph_ax2.plot([], [], 'r-', label='Y')
    dashboard.graph_line_spy, = dashboard.graph_ax2.plot([], [], 'k-', label='SPY')
    
    # Configure graph axes
    dashboard.graph_ax1.set_xlabel('')
    dashboard.graph_ax1.set_ylabel('X and SPX')
    dashboard.graph_ax1.legend()
    dashboard.graph_ax1.grid(True)
    
    dashboard.graph_ax2.set_xlabel('Time [s]')
    dashboard.graph_ax2.set_ylabel('Y and SPY')
    dashboard.graph_ax2.legend()
    dashboard.graph_ax2.grid(True)
    
    # Adjust spacing between subplots
    dashboard.graph_fig.subplots_adjust(left=0.12, right=0.95, hspace=0.3, bottom=0.15, top=0.95)
    
    # Create graph canvas
    dashboard.graph_canvas = FigureCanvasTkAgg(dashboard.graph_fig, dashboard.graph_plot_frame)
    dashboard.graph_canvas.draw()
    dashboard.graph_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

def toggle_team_view(dashboard):
    """Toggle between Team 1 and Team 2 map views"""
    # Toggle the current team
    dashboard.current_team = 3 - dashboard.current_team  # Switches between 1 and 2
    
    # Remove the current image
    dashboard.field_img.remove()
    
    # Load image using the stored path
    img = plt.imread(dashboard.map_image_path)
    
    if dashboard.current_team == 1:
        # Team 1 view (origin at bottom left)
        # Display image with Team 1 orientation
        dashboard.field_img = dashboard.ax.imshow(img, extent=[1500, 0, 0, 800])
        
        # Set axis orientation for Team 1
        dashboard.ax.set_xlim(1500, 0)  # X-axis goes from 1500 (left) to 0 (right)
        dashboard.ax.set_ylim(0, 800)    # Y-axis goes from 0 (bottom) to 800 (top)
        
        # Move y-axis to right side for Team 1
        dashboard.ax.yaxis.set_label_position("right")
        dashboard.ax.yaxis.tick_right()
        
        # Update button text
        dashboard.team_toggle_btn.config(text="Current: Team 1", fg="#FF0000")
    else:
        # Team 2 view (origin at top left)
        # Display image with Team 2 orientation (same image, different extent)
        dashboard.field_img = dashboard.ax.imshow(img, extent=[0, 1500, 800, 0])
        
        # Set axis orientation for Team 2
        dashboard.ax.set_xlim(0, 1500)  # X-axis goes from 0 (left) to 1500 (right)  
        dashboard.ax.set_ylim(800, 0)    # Y-axis goes from 800 (top) to 0 (bottom)
        
        # Move y-axis to left side for Team 2
        dashboard.ax.yaxis.set_label_position("left")
        dashboard.ax.yaxis.tick_left()
        
        # Update button text
        dashboard.team_toggle_btn.config(text="Current: Team 2", fg="#0000FF")
    
    # Update robot positions based on the current view
    update_robot_display(dashboard)
    
    # Ensure grid is visible
    dashboard.ax.grid(True)
    
    # Keep ticks consistent
    x_ticks = [0, 250, 500, 750, 1000, 1250, 1500]
    dashboard.ax.set_xticks(x_ticks)
    
    # Redraw the canvas
    dashboard.fig.canvas.draw_idle()
    dashboard.fig.canvas.flush_events()

def update_robot_display(dashboard):
    """Update the robot markers on the map based on the current team view"""
    # Use the stored robot positions to update the display
    if hasattr(dashboard, 'robot1_pos_data') and hasattr(dashboard, 'robot2_pos_data'):
        r1x, r1y = dashboard.robot1_pos_data
        r2x, r2y = dashboard.robot2_pos_data
        
        # No coordinate transformation needed - just update the plot data
        dashboard.robot1_pos.set_data(r1x, r1y)
        dashboard.robot2_pos.set_data(r2x, r2y)

def toggle_graph_pause(dashboard):
    """Toggle graph pause state"""
    dashboard.graph_paused = not dashboard.graph_paused
    
    if dashboard.graph_paused:
        dashboard.pause_button.config(text="Resume")
        dashboard.graph_status_label.config(text="Status: Paused")
    else:
        dashboard.pause_button.config(text="Pause")
        dashboard.graph_status_label.config(text="Status: Running")

def reset_graph_time(dashboard):
    """Reset graph time to zero and clear existing data"""
    # Store the pause state
    was_paused = dashboard.graph_paused
    
    # Temporarily pause if it wasn't already
    dashboard.graph_paused = True
    
    # Clear data
    dashboard.graph_time_data = []
    dashboard.graph_x_data = []
    dashboard.graph_y_data = []
    dashboard.graph_spx_data = []
    dashboard.graph_spy_data = []
    
    # Reset the graph start time to current time 
    # (will be properly set when next data point arrives)
    if hasattr(dashboard, 'node'):
        dashboard.graph_start_time = dashboard.node.get_clock().now().nanoseconds / 1e9
    
    # Update the time display
    dashboard.graph_time_label.config(text="Elapsed: 0.00s")
    
    # Update graph
    update_graph(dashboard)
    
    # Restore previous pause state
    dashboard.graph_paused = was_paused
    
    # Update button text
    if dashboard.graph_paused:
        dashboard.pause_button.config(text="Resume")
        dashboard.graph_status_label.config(text="Status: Paused")
    else:
        dashboard.pause_button.config(text="Pause")
        dashboard.graph_status_label.config(text="Status: Running")

def update_graph(dashboard):
    """Update the graph with the latest data while keeping setpoints centered"""
    # Check if data exists
    if not hasattr(dashboard, 'graph_time_data') or len(dashboard.graph_time_data) == 0:
        return
    
    try:
        # Update the time display if there's data
        if dashboard.graph_time_data:
            current_time = dashboard.graph_time_data[-1]
            dashboard.graph_time_label.config(text=f"Elapsed: {current_time:.2f}s")
            
        # Clear both axes to start fresh
        dashboard.graph_ax1.clear()
        dashboard.graph_ax2.clear()
        
        # Replot the data on both axes
        dashboard.graph_ax1.plot(dashboard.graph_time_data, dashboard.graph_x_data, 'b-', label='X')
        dashboard.graph_ax1.plot(dashboard.graph_time_data, dashboard.graph_spx_data, 'k-', label='SPX')
        dashboard.graph_ax2.plot(dashboard.graph_time_data, dashboard.graph_y_data, 'r-', label='Y')
        dashboard.graph_ax2.plot(dashboard.graph_time_data, dashboard.graph_spy_data, 'k-', label='SPY')
        
        # Update x-axis limits (sliding window of 10 seconds)
        if dashboard.graph_time_data:
            current_time = dashboard.graph_time_data[-1]
            min_time = max(0, current_time - 10)
            
            dashboard.graph_ax1.set_xlim(min_time, current_time + 1)
            dashboard.graph_ax2.set_xlim(min_time, current_time + 1)
        
        # Center the y-axis around the setpoints (SPX and SPY) if auto-center is enabled
        if hasattr(dashboard, 'auto_center_var') and dashboard.auto_center_var.get():
            if dashboard.graph_spx_data and dashboard.graph_spy_data:
                # Get the current SPX and SPY values
                current_spx = dashboard.graph_spx_data[-1]
                current_spy = dashboard.graph_spy_data[-1]
                
                # Calculate min and max for X axis (actual position)
                if dashboard.graph_x_data:
                    # Filter to only recent data (within the current view)
                    recent_indices = [i for i, t in enumerate(dashboard.graph_time_data) if t >= min_time]
                    recent_x_data = [dashboard.graph_x_data[i] for i in recent_indices]
                    
                    if recent_x_data:
                        x_min = min(recent_x_data)
                        x_max = max(recent_x_data)
                        
                        # Calculate the range and make sure it's at least 50 units
                        x_range = max(50, x_max - x_min)
                        
                        # Make sure SPX is in the middle with enough margin above and below
                        margin = max(abs(current_spx - x_min), abs(x_max - current_spx), x_range/2)
                        dashboard.graph_ax1.set_ylim(
                            current_spx - margin - 10,  # Add 10 extra units for padding
                            current_spx + margin + 10
                        )
                
                # Calculate min and max for Y axis (actual position)
                if dashboard.graph_y_data:
                    # Filter to only recent data (within the current view)
                    recent_indices = [i for i, t in enumerate(dashboard.graph_time_data) if t >= min_time]
                    recent_y_data = [dashboard.graph_y_data[i] for i in recent_indices]
                    
                    if recent_y_data:
                        y_min = min(recent_y_data)
                        y_max = max(recent_y_data)
                        
                        # Calculate the range and make sure it's at least 50 units
                        y_range = max(50, y_max - y_min)
                        
                        # Make sure SPY is in the middle with enough margin above and below
                        margin = max(abs(current_spy - y_min), abs(y_max - current_spy), y_range/2)
                        dashboard.graph_ax2.set_ylim(
                            current_spy - margin - 10,  # Add 10 extra units for padding
                            current_spy + margin + 10
                        )
                        
                # Add horizontal lines for setpoints to make them more visible
                dashboard.graph_ax1.axhline(y=current_spx, color='k', linestyle='--', alpha=0.5)
                dashboard.graph_ax2.axhline(y=current_spy, color='k', linestyle='--', alpha=0.5)
        
        # Reconfigure graph axes
        dashboard.graph_ax1.set_xlabel('')
        dashboard.graph_ax1.set_ylabel('X and SPX')
        dashboard.graph_ax1.legend()
        dashboard.graph_ax1.grid(True)
        
        dashboard.graph_ax2.set_xlabel('Time [s]')
        dashboard.graph_ax2.set_ylabel('Y and SPY')
        dashboard.graph_ax2.legend()
        dashboard.graph_ax2.grid(True)
        
        # Adjust spacing between subplots
        dashboard.graph_fig.subplots_adjust(hspace=0.3, bottom=0.15)  # Increase bottom margin
        
        # Adjust figure layout for better spacing
        dashboard.graph_fig.tight_layout(pad=2.0)
        
        # Redraw the canvas
        dashboard.graph_canvas.draw()
        dashboard.graph_canvas.flush_events()
    
    except Exception as e:
        print(f"Error updating graph: {e}")
        
# Method to add to ROS2_Dashboard class
def add_graph_methods(dashboard):
    """Add graph-related methods to the dashboard"""
    # Add graph update method
    dashboard.update_graph = lambda: update_graph(dashboard)
    dashboard.toggle_graph_pause = lambda: toggle_graph_pause(dashboard)
    dashboard.reset_graph_time = lambda: reset_graph_time(dashboard)