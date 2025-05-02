import tkinter as tk
from tkinter import ttk

def create_send_data_widget(dashboard):
    """Create the send data widget"""
    # Send Data widget
    dashboard.send_data_frame = tk.LabelFrame(dashboard.main_frame, text="SEND DATA", padx=5, pady=5)
    dashboard.send_data_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
    
    # Configure grid to make frame expandable
    dashboard.send_data_frame.grid_columnconfigure(0, weight=1)
    
    # Reduce vertical spacing between elements
    tk.Label(dashboard.send_data_frame, text="APP \u2192 /Topic").grid(
        row=0, column=0, sticky="w", pady=(0,2), padx=5
    )
    dashboard.input_entry = tk.Entry(dashboard.send_data_frame)
    dashboard.input_entry.grid(
        row=1, column=0, sticky="ew", padx=5, pady=(0,5)
    )
    
    # Bind Enter key to send_data function
    dashboard.input_entry.bind("<Return>", lambda event: dashboard.send_data())
    
    # Topic output selection
    tk.Label(dashboard.send_data_frame, text="/Topic output").grid(
        row=2, column=0, sticky="w", pady=(0,2), padx=5
    )
    dashboard.output_topics = ttk.Combobox(dashboard.send_data_frame)
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
        dashboard.send_data_frame, 
        text="SEND", 
        command=dashboard.send_data
    )
    dashboard.send_button.grid(
        row=4, column=0, pady=5
    )
    
    # Feedback label
    dashboard.feedback_label = tk.Label(
        dashboard.send_data_frame, 
        text="", 
        wraplength=250, 
        fg="dark green"
    )
    dashboard.feedback_label.grid(
        row=5, column=0, pady=(0,2)
    )

def update_output_topic(dashboard):
    """Update the output topic when selected from the combobox"""
    topic = dashboard.output_topics.get()
    if topic:
        dashboard.output_topic = topic
        # Save configuration immediately when output topic changes
        dashboard.save_config()