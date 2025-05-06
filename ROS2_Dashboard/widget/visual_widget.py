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
    # Topic selection for robots' positions
    map_topics_frame = tk.Frame(dashboard.map_tab)
    map_topics_frame.pack(fill=tk.X, pady=2)
    
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
        img = plt.imread("/home/haidar/ros2_git/src/ROS2-Dashboard/ROS2_Dashboard/resources/new_map.png")
        flipped_img = np.flipud(img)  # Flip the image vertically
        dashboard.field_img = dashboard.ax.imshow(flipped_img, extent=[0, 1500, 0, 800])  # Set map scale to 1500 x 800
        
        # Set up plot
        dashboard.ax.set_xlim(0, 1500)  # Set x-axis range
        dashboard.ax.set_ylim(800, 0)   # Set y-axis range (invert y-axis for display)
        dashboard.ax.grid(True)

        # Initialize robot positions (start at origin)
        dashboard.robot2_pos = dashboard.ax.plot(0, 0, marker='o', mfc='#0000FF', mec='#000000', mew=1.5 , markersize=40)[0]  # Biru muda untuk Robot 2

        # Create canvas
        dashboard.canvas = FigureCanvasTkAgg(dashboard.fig, dashboard.map_tab)
        dashboard.canvas.draw()
        dashboard.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
    except Exception as e:
        print(f"Error loading field map: {e}")
        tk.Label(dashboard.map_tab, text="Error loading map").pack()
    
    # ---- GRAPH TAB ----
    # Graph topic selection frame
    graph_topics_frame = tk.Frame(dashboard.graph_tab)
    graph_topics_frame.pack(fill=tk.X, pady=2)
    
    # Graph topic selection
    tk.Label(graph_topics_frame, text="Graph Topic:").grid(row=0, column=0, sticky="w", padx=2)
    dashboard.graph_topic_combo = ttk.Combobox(graph_topics_frame, values=dashboard.available_topics, width=50)
    dashboard.graph_topic_combo.grid(row=0, column=1, padx=2, pady=2)
    dashboard.graph_topic_combo.bind("<<ComboboxSelected>>", 
                        lambda e: dashboard.subscribe_to_graph_topic(dashboard.graph_topic_combo.get()))
    
    # Create matplotlib figure for graph
    dashboard.graph_fig = Figure(figsize=(2, 4), dpi=100)
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
    dashboard.graph_fig.subplots_adjust(hspace=0.3)  # Reduce vertical spacing
    
    # Create graph canvas
    dashboard.graph_canvas = FigureCanvasTkAgg(dashboard.graph_fig, dashboard.graph_tab)
    dashboard.graph_canvas.draw()
    dashboard.graph_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
 
def update_graph(dashboard):
    """Update the graph with the latest data"""
    # Check if data exists
    if not hasattr(dashboard, 'graph_time_data') or len(dashboard.graph_time_data) == 0:
        return
    
    try:
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
            
            dashboard.graph_ax1.set_xlim(min_time, current_time)
            dashboard.graph_ax2.set_xlim(min_time, current_time)
        
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
        dashboard.graph_fig.subplots_adjust(hspace=0.3)  # Reduce vertical spacing
        
        # Adjust figure layout for better spacing
        dashboard.graph_fig.tight_layout()
        
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