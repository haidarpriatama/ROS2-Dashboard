import tkinter as tk
from tkinter import ttk

def create_data_logger_widget(dashboard):
    """Create the data logger widget"""
    # Data Logger widget
    dashboard.data_logger_frame = tk.LabelFrame(dashboard.main_frame, text="DATA LOGGER", padx=5, pady=5)
    dashboard.data_logger_frame.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)
    
    # Configure grid weights to make frame expandable
    dashboard.data_logger_frame.grid_columnconfigure(0, weight=1)
    dashboard.data_logger_frame.grid_rowconfigure(0, weight=1)
    
    # Create a main frame inside the labeled frame
    main_frame = tk.Frame(dashboard.data_logger_frame)
    main_frame.pack(fill=tk.BOTH, expand=True)
    main_frame.grid_columnconfigure(0, weight=1)
    main_frame.grid_rowconfigure(0, weight=1)
    
    # Create a canvas with scrollbar for vertical scrolling
    canvas = tk.Canvas(main_frame, highlightthickness=0)
    vsb = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
    
    # Configure the canvas
    canvas.configure(yscrollcommand=vsb.set)
    
    # Position canvas and scrollbar with pack to ensure they fill properly
    vsb.pack(side="right", fill="y")
    canvas.pack(side="left", fill="both", expand=True)
    
    # Create frame inside canvas to hold all the topic widgets
    scrollable_frame = tk.Frame(canvas)
    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )
    scrollable_frame.columnconfigure(0, weight=1)
    
    # Create window inside canvas to display the scrollable frame
    canvas_window = canvas.create_window((0, 0), window=scrollable_frame, anchor="nw", tags="frame")
    
    # Make sure the scrollable frame expands to fill canvas width
    def on_canvas_configure(event):
        # Update the width of the canvas window to fill canvas
        canvas.itemconfig("frame", width=event.width)
    
    canvas.bind("<Configure>", on_canvas_configure)
    
    # Create and add the topics (1-6) ubah (1, #) kalo mau nambah
    for i in range(1, 8):
        # Create a frame for each topic section (fills canvas width)
        topic_section = tk.Frame(scrollable_frame)
        topic_section.pack(fill=tk.X, expand=True, pady=2)
        topic_section.columnconfigure(0, weight=1)
        
        # Topic label and combobox row
        topic_header = tk.Frame(topic_section)
        topic_header.pack(fill=tk.X, expand=True)
        
        # Topic label
        topic_label = tk.Label(topic_header, text=f"Topic {i}:", anchor="w")
        topic_label.pack(side=tk.LEFT, padx=2, pady=2)
        
        # Topic combobox
        combo = ttk.Combobox(topic_section, values=dashboard.available_topics)
        combo.pack(fill=tk.X, padx=2, pady=2)
        setattr(dashboard, f"topic{i}_combo", combo)
        
        # Set default value
        if hasattr(dashboard, f"topic{i}") and getattr(dashboard, f"topic{i}"):
            combo.set(getattr(dashboard, f"topic{i}"))
        else:
            combo.set("Select Topic")
        
        # Text widget container frame
        text_container = tk.Frame(topic_section, borderwidth=1, relief=tk.SUNKEN)
        text_container.pack(fill=tk.X, expand=True, padx=2, pady=2)
        text_container.columnconfigure(0, weight=1)

        # Text widget with horizontal scrollbar
        text_widget = tk.Text(text_container, height=10, wrap="none")  # Increase height to 10
        text_widget.grid(row=0, column=0, sticky="nsew")

        # Add horizontal scrollbar
        h_scrollbar = ttk.Scrollbar(text_container, orient=tk.HORIZONTAL, command=text_widget.xview)
        h_scrollbar.grid(row=1, column=0, sticky="ew")
        text_widget.configure(xscrollcommand=h_scrollbar.set)

        # Store text widget reference
        setattr(dashboard, f"topic{i}_text", text_widget)
        text_widget.config(state=tk.DISABLED)
        
        # Bind topic selection
        combo.bind(
            "<<ComboboxSelected>>", 
            lambda e, idx=i: dashboard.subscribe_to_topic(idx, getattr(dashboard, f"topic{idx}_combo").get())
        )
        
        # Initialize topic data list
        if not hasattr(dashboard, f"topic{i}_data"):
            setattr(dashboard, f"topic{i}_data", [])