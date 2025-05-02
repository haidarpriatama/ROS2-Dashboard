#!/usr/bin/env python3
import tkinter as tk
import rclpy
from dashboard import ROS2_Dashboard

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create tkinter root and application
    root = tk.Tk()
    app = ROS2_Dashboard(root)
    
    # Handle window close properly
    root.protocol("WM_DELETE_WINDOW", app.exit_application)
    
    # Start main loop
    root.mainloop()

if __name__ == "__main__":
    main()