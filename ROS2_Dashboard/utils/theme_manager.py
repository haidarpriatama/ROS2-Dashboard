import tkinter as tk
from tkinter import ttk

class ThemeManager:
    """Manages theme settings for the ROS2 Dashboard"""
    
    # Define color schemes
    THEMES = {
        "light": {
            "bg": "#f0f0f0",  # Light gray background
            "fg": "#000000",  # Black text
            "frame_bg": "#ffffff",  # White frame background
            "label_bg": "#f0f0f0",  # Light gray label background
            "label_fg": "#000000",  # Black label text
            "button_bg": "#e0e0e0",  # Light gray button background
            "button_fg": "#000000",  # Black button text
            "entry_bg": "#ffffff",  # White entry background
            "entry_fg": "#000000",  # Black entry text
            "text_bg": "#ffffff",  # White text widget background
            "text_fg": "#000000",  # Black text widget text
            "highlight_bg": "#4a90e2",  # Blue highlight
            "highlight_fg": "#ffffff",  # White highlight text
            "canvas_bg": "#ffffff",  # White canvas background
            "scrollbar_bg": "#e0e0e0",  # Light gray scrollbar background
            "scrollbar_fg": "#c0c0c0",  # Darker gray scrollbar foreground
            "progress_bg": "#ffffff",  # White progress bar background
            "progress_fg": "#4caf50",  # Green progress bar foreground
            "tab_bg": "#e0e0e0",  # Light gray tab background
            "tab_fg": "#000000",  # Black tab text
            "label_frame_fg": "#000000"  # Black labelframe text
        },
        "dark": {
            "bg": "#2e2e2e",  # Dark gray background
            "fg": "#ffffff",  # White text
            "frame_bg": "#333333",  # Slightly lighter frame background
            "label_bg": "#2e2e2e",  # Dark gray label background
            "label_fg": "#ffffff",  # White label text
            "button_bg": "#444444",  # Medium gray button background
            "button_fg": "#ffffff",  # White button text
            "entry_bg": "#3c3c3c",  # Medium gray entry background
            "entry_fg": "#ffffff",  # White entry text
            "text_bg": "#3c3c3c",  # Medium gray text widget background
            "text_fg": "#ffffff",  # White text widget text
            "highlight_bg": "#2196f3",  # Blue highlight
            "highlight_fg": "#ffffff",  # White highlight text
            "canvas_bg": "#3c3c3c",  # Medium gray canvas background
            "scrollbar_bg": "#444444",  # Medium gray scrollbar background
            "scrollbar_fg": "#666666",  # Lighter gray scrollbar foreground
            "progress_bg": "#3c3c3c",  # Medium gray progress bar background
            "progress_fg": "#4caf50",  # Green progress bar foreground
            "tab_bg": "#444444",  # Medium gray tab background
            "tab_fg": "#ffffff",  # White tab text
            "label_frame_fg": "#ffffff"  # White labelframe text
        }
    }
    
    @classmethod
    def apply_theme(cls, dashboard, theme_name="light"):
        """Apply the specified theme to all widgets"""
        if theme_name not in cls.THEMES:
            print(f"Theme {theme_name} not found. Using light theme.")
            theme_name = "light"
        
        dashboard.current_theme = theme_name
        theme = cls.THEMES[theme_name]
        
        # Configure styles
        style = ttk.Style()
        style.theme_use('default')
        
        # Configure TTK styles
        # Note: ttk widgets don't use bg and fg directly, they use style configuration
        style.configure("TFrame", background=theme["frame_bg"])
        style.configure("TLabel", background=theme["label_bg"], foreground=theme["label_fg"])
        style.configure("TButton", background=theme["button_bg"], foreground=theme["button_fg"])
        style.configure("TEntry", fieldbackground=theme["entry_bg"], foreground=theme["entry_fg"])
        style.configure("TCombobox", fieldbackground=theme["entry_bg"], foreground=theme["entry_fg"], 
                        selectbackground=theme["highlight_bg"], selectforeground=theme["highlight_fg"])
        style.configure("TScrollbar", background=theme["scrollbar_bg"], troughcolor=theme["scrollbar_bg"], 
                        arrowcolor=theme["scrollbar_fg"])
        style.configure("TNotebook", background=theme["tab_bg"], foreground=theme["tab_fg"])
        style.configure("TNotebook.Tab", background=theme["tab_bg"], foreground=theme["tab_fg"])
        
        # Configure progress bar style
        style.configure("green.Horizontal.TProgressbar", 
                        troughcolor=theme["progress_bg"], 
                        background=theme["progress_fg"])
        
        # Configure root and main frame
        dashboard.master.configure(bg=theme["bg"])
        dashboard.main_frame.configure(bg=theme["bg"])
        
        # Configure labelframes
        for attr_name in ["data_logger_frame", "send_data_frame", "status_frame", "position_frame"]:
            if hasattr(dashboard, attr_name):
                getattr(dashboard, attr_name).configure(bg=theme["frame_bg"], fg=theme["label_frame_fg"])
        
        # Configure frames (only tk.Frame, not ttk.Frame)
        for attr_name in ["robots_frame", "robot1_frame", "robot2_frame", "gui_info_frame", 
                          "time_date_frame"]:
            if hasattr(dashboard, attr_name):
                widget = getattr(dashboard, attr_name)
                if isinstance(widget, tk.Frame):  # Make sure it's a tk.Frame, not ttk.Frame
                    widget.configure(bg=theme["frame_bg"])
        
        # Handle ttk.Frame widgets specially (like map_tab and graph_tab)
        # ttk widgets don't use direct configure with bg/fg, so we skip them
        
        # Configure text widgets
        for i in range(1, 8):
            text_widget_name = f"topic{i}_text"
            if hasattr(dashboard, text_widget_name):
                text_widget = getattr(dashboard, text_widget_name)
                text_widget.configure(bg=theme["text_bg"], fg=theme["text_fg"])
        
        # Configure buttons
        for attr_name in ["send_button", "exit_button", "settings_button"]:
            if hasattr(dashboard, attr_name):
                widget = getattr(dashboard, attr_name)
                if isinstance(widget, tk.Button):  # Ensure it's a tk Button not ttk
                    widget.configure(bg=theme["button_bg"], fg=theme["button_fg"])
        
        # Configure labels
        for attr_name in dir(dashboard):
            if attr_name.endswith("_label") or attr_name in ["robot1_coords", "robot2_coords"]:
                try:
                    widget = getattr(dashboard, attr_name)
                    if isinstance(widget, tk.Label):  # Ensure it's a tk Label not ttk
                        widget.configure(bg=theme["label_bg"], fg=theme["label_fg"])
                except Exception as e:
                    print(f"Error configuring {attr_name}: {e}")
        
        # Configure entries
        if hasattr(dashboard, "input_entry"):
            if isinstance(dashboard.input_entry, tk.Entry):  # Ensure it's a tk Entry not ttk
                dashboard.input_entry.configure(bg=theme["entry_bg"], fg=theme["entry_fg"])
        
        # Update chart backgrounds
        if hasattr(dashboard, "fig") and hasattr(dashboard, "ax"):
            dashboard.fig.set_facecolor(theme["canvas_bg"])
            dashboard.ax.set_facecolor(theme["canvas_bg"])
            dashboard.ax.tick_params(colors=theme["label_fg"])
            dashboard.ax.xaxis.label.set_color(theme["label_fg"])
            dashboard.ax.yaxis.label.set_color(theme["label_fg"])
            dashboard.ax.spines['bottom'].set_color(theme["label_fg"])
            dashboard.ax.spines['top'].set_color(theme["label_fg"])
            dashboard.ax.spines['right'].set_color(theme["label_fg"])
            dashboard.ax.spines['left'].set_color(theme["label_fg"])
            dashboard.ax.grid(True, color=theme["scrollbar_fg"])
            if hasattr(dashboard, "canvas"):
                dashboard.canvas.draw()
        
        # Update graph chart backgrounds
        if hasattr(dashboard, "graph_fig"):
            dashboard.graph_fig.set_facecolor(theme["canvas_bg"])
            if hasattr(dashboard, "graph_ax1") and hasattr(dashboard, "graph_ax2"):
                for ax in [dashboard.graph_ax1, dashboard.graph_ax2]:
                    ax.set_facecolor(theme["canvas_bg"])
                    ax.tick_params(colors=theme["label_fg"])
                    ax.xaxis.label.set_color(theme["label_fg"])
                    ax.yaxis.label.set_color(theme["label_fg"])
                    ax.spines['bottom'].set_color(theme["label_fg"])
                    ax.spines['top'].set_color(theme["label_fg"])
                    ax.spines['right'].set_color(theme["label_fg"])
                    ax.spines['left'].set_color(theme["label_fg"])
                    ax.grid(True, color=theme["scrollbar_fg"])
                if hasattr(dashboard, "graph_canvas"):
                    dashboard.graph_canvas.draw()
        
        # Update all child widgets recursively
        cls._update_widgets_recursive(dashboard.main_frame, theme)
    
    @classmethod
    def _update_widgets_recursive(cls, widget, theme):
        """Recursively update all widgets with appropriate theme colors"""
        for child in widget.winfo_children():
            try:
                # Handle different widget types
                if isinstance(child, tk.Frame) and not isinstance(child, ttk.Frame):
                    child.configure(bg=theme["frame_bg"])
                    
                elif isinstance(child, tk.LabelFrame):
                    child.configure(bg=theme["frame_bg"], fg=theme["label_frame_fg"])
                    
                elif isinstance(child, tk.Label):
                    child.configure(bg=theme["label_bg"], fg=theme["label_fg"])
                    
                elif isinstance(child, tk.Button):
                    child.configure(bg=theme["button_bg"], fg=theme["button_fg"])
                    
                elif isinstance(child, tk.Entry):
                    child.configure(bg=theme["entry_bg"], fg=theme["entry_fg"])
                    
                elif isinstance(child, tk.Text):
                    child.configure(bg=theme["text_bg"], fg=theme["text_fg"])
                    
                # Skip ttk widgets as they don't support direct bg/fg configuration
                
                # Apply to children of this widget
                cls._update_widgets_recursive(child, theme)
                
            except Exception as e:
                # Skip widget if it can't be configured
                print(f"Error updating widget {child}: {e}")
                pass