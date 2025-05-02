import tkinter as tk
from PIL import Image, ImageTk

def create_gui_info_widget(dashboard):
    """Create the GUI info sidebar widget"""
    # GUI Info sidebar
    dashboard.gui_info_frame = tk.Frame(dashboard.main_frame)
    dashboard.gui_info_frame.grid(row=0, column=3, rowspan=2, sticky="nsew", padx=5, pady=5)
    
    # App info
    tk.Label(
        dashboard.gui_info_frame, 
        text="Graphical\nUser\nInterface\nRobot", 
        font=("Arial", 14) 
    ).pack(pady=20)
    
    # Load logos
    try:
        # Logo 1
        logo1_path = "/home/haidar/ros2_git/src/ROS2-Dashboard/ROS2_Dashboard/resources/UNDIP_logo.png"
        logo1_img = Image.open(logo1_path)
        logo1_img = logo1_img.resize((100, 100), Image.LANCZOS)
        logo1_photo = ImageTk.PhotoImage(logo1_img)
        
        logo1_label = tk.Label(dashboard.gui_info_frame, image=logo1_photo)
        logo1_label.image = logo1_photo  # Keep a reference to prevent garbage collection
        logo1_label.pack(pady=5)
        
        # Logo 2
        logo2_path = "/home/haidar/ros2_git/src/ROS2-Dashboard/ROS2_Dashboard/resources/bandha_logo.png"
        logo2_img = Image.open(logo2_path)
        logo2_img = logo2_img.resize((100, 100), Image.LANCZOS)
        logo2_photo = ImageTk.PhotoImage(logo2_img)
        
        logo2_label = tk.Label(dashboard.gui_info_frame, image=logo2_photo)
        logo2_label.image = logo2_photo  # Keep a reference to prevent garbage collection
        logo2_label.pack(pady=5)
    except Exception as e:
        print(f"Error loading logos: {e}")
        # Fallback if images can't be loaded
        dashboard.logo_frame1 = tk.LabelFrame(dashboard.gui_info_frame, text="LOGO", width=100, height=100)
        dashboard.logo_frame1.pack(pady=10)
        
        dashboard.logo_frame2 = tk.LabelFrame(dashboard.gui_info_frame, text="LOGO", width=100, height=100)
        dashboard.logo_frame2.pack(pady=10)
    
    # Time and date moved here (above config icon)
    dashboard.time_date_frame = tk.Frame(dashboard.gui_info_frame)
    dashboard.time_date_frame.pack(fill=tk.X, pady=5)
    
    time_frame = tk.Frame(dashboard.time_date_frame)
    time_frame.pack(fill=tk.X, pady=2)
    # tk.Label(time_frame, text="Time :", anchor="e", width=6).pack(side=tk.LEFT)
    dashboard.time_label = tk.Label(time_frame, text="HH:MM", anchor="w")
    dashboard.time_label.pack(side=tk.TOP, padx=5)
    
    date_frame = tk.Frame(dashboard.time_date_frame)
    date_frame.pack(fill=tk.X, pady=2)
    # tk.Label(date_frame, text="Date :", anchor="e", width=6).pack(side=tk.LEFT)
    dashboard.date_label = tk.Label(date_frame, text="DD/MM/YYYY", anchor="w")
    dashboard.date_label.pack(side=tk.TOP, padx=5)
    
    # Control buttons
    dashboard.exit_button = tk.Button(dashboard.gui_info_frame, text="EXIT", command=dashboard.exit_application, width=10)
    dashboard.exit_button.pack(side=tk.BOTTOM, pady=5)

    # Settings button with icon
    try:
        icon_path = "/home/haidar/ros2_git/src/ROS2-Dashboard/ROS2_Dashboard/resources/icon_config.png"
        settings_icon = Image.open(icon_path)
        settings_icon = settings_icon.resize((24, 24), Image.LANCZOS)
        settings_photo = ImageTk.PhotoImage(settings_icon)
        
        dashboard.settings_button = tk.Button(dashboard.gui_info_frame, image=settings_photo, command=dashboard.open_settings)
        dashboard.settings_button.image = settings_photo  # Keep a reference to prevent garbage collection
        dashboard.settings_button.pack(side=tk.BOTTOM, pady=5)
    except Exception as e:
        print(f"Error loading settings icon: {e}")
        # Fallback to text button
        dashboard.settings_button = tk.Button(dashboard.gui_info_frame, text="⚙️", command=dashboard.open_settings, font=("Arial", 12))
        dashboard.settings_button.pack(side=tk.BOTTOM, pady=5)