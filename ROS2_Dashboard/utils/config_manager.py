import os
import json
import tkinter as tk

def save_config(dashboard):
    """Save configuration to a JSON file"""
    # Create configuration dictionary with only necessary items
    # Exclude auto-subscribed status and battery topics
    config = {
        # Data logger topics
        "topic1": dashboard.topic1,
        "topic2": dashboard.topic2,
        "topic3": dashboard.topic3,
        "topic4": dashboard.topic4,
        "topic5": dashboard.topic5,
        "topic6": dashboard.topic6,
        
        # Position topics - these need to be saved
        "pos_topic1": getattr(dashboard, "pos_topic1", None),  # Robot 1 position
        "pos_topic2": getattr(dashboard, "pos_topic2", None),  # Robot 2 position
        
        # Output and graph topics
        "output_topic": getattr(dashboard, "output_topic", None),
        "graph_topic": getattr(dashboard, "graph_topic", None),
        
        # PID settings for Robot 1
        "pid_p1_value": getattr(dashboard, "pid_p1_value", None),
        "pid_i1_value": getattr(dashboard, "pid_i1_value", None),
        "pid_d1_value": getattr(dashboard, "pid_d1_value", None),
        
        # PID settings for Robot 2
        "pid_p2_value": getattr(dashboard, "pid_p2_value", None),
        "pid_i2_value": getattr(dashboard, "pid_i2_value", None),
        "pid_d2_value": getattr(dashboard, "pid_d2_value", None),
    }
    
    # Remove NULL/None values to make the config file cleaner
    clean_config = {k: v for k, v in config.items() if v is not None}
    
    # Save to file
    try:
        # Ensure directory exists
        config_dir = os.path.dirname(dashboard.config_file)
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)
            
        # Save with nice formatting
        with open(dashboard.config_file, 'w') as f:
            json.dump(clean_config, f, indent=4)
            
        print(f"Configuration saved to {dashboard.config_file}")
        
        # Print saved topics
        for i in range(1, 7):
            if clean_config.get(f'topic{i}'):
                print(f"Topic {i} saved as: {clean_config[f'topic{i}']}")
        
        if clean_config.get('output_topic'):
            print(f"Output topic saved as: {clean_config['output_topic']}")
        
        if clean_config.get('graph_topic'):
            print(f"Graph topic saved as: {clean_config['graph_topic']}")
            
        if clean_config.get('pos_topic1'):
            print(f"Robot 1 position topic saved as: {clean_config['pos_topic1']}")
            
        if clean_config.get('pos_topic2'):
            print(f"Robot 2 position topic saved as: {clean_config['pos_topic2']}")
        
        # Print PID settings for Robot 1
        if all(key in clean_config for key in ["pid_p1_value", "pid_i1_value", "pid_d1_value"]):
            print(f"Robot 1 PID settings saved: P={clean_config['pid_p1_value']}, I={clean_config['pid_i1_value']}, D={clean_config['pid_d1_value']}")
        
        # Print PID settings for Robot 2
        if all(key in clean_config for key in ["pid_p2_value", "pid_i2_value", "pid_d2_value"]):
            print(f"Robot 2 PID settings saved: P={clean_config['pid_p2_value']}, I={clean_config['pid_i2_value']}, D={clean_config['pid_d2_value']}")
    except Exception as e:
        print(f"Error saving configuration: {e}")

def load_config(dashboard):
    """Load configuration from a JSON file"""
    try:
        if os.path.exists(dashboard.config_file):
            with open(dashboard.config_file, 'r') as f:
                config = json.load(f)
            
            # Print loaded configuration for debugging
            print(f"Loaded configuration from {dashboard.config_file}")
            
            # Apply configuration to topics (1-6)
            for i in range(1, 7):  # Now supporting 6 topics
                topic_key = f"topic{i}"
                if topic_key in config and config[topic_key]:
                    if config[topic_key] in dashboard.available_topics:
                        dashboard.subscribe_to_topic(i, config[topic_key])
                        # Also update combobox in main UI
                        if hasattr(dashboard, f"topic{i}_combo"):
                            getattr(dashboard, f"topic{i}_combo").set(config[topic_key])
            
            # Auto-subscribe to status topics for both robots
            for i in range(1, 3):
                # Try to auto-subscribe to ROBOT1/STATUS or ROBOT2/STATUS
                default_topic = f"/ROBOT{i}/STATUS"
                if default_topic in dashboard.available_topics:
                    dashboard.subscribe_to_status(default_topic, i)
                    print(f"Auto-subscribed to {default_topic}")
                
                # Auto-subscribe to battery topics
                battery_topic = f"/ROBOT{i}/battery"
                if battery_topic in dashboard.available_topics:
                    dashboard.subscribe_to_battery(battery_topic, i)
                    print(f"Auto-subscribed to {battery_topic}")
            
            # Load position topics for both robots
            for i in range(1, 3):
                pos_key = f"pos_topic{i}"
                if pos_key in config and config[pos_key]:
                    if config[pos_key] in dashboard.available_topics:
                        dashboard.subscribe_to_position(config[pos_key], i)
                        print(f"Loaded Robot {i} position topic: {config[pos_key]}")
            
            # Load output topic
            if "output_topic" in config and config["output_topic"]:
                dashboard.output_topic = config["output_topic"]
                if hasattr(dashboard, "output_topics"):
                    dashboard.output_topics.set(dashboard.output_topic)
                print(f"Loaded output topic: {dashboard.output_topic}")
            
            # Load graph topic
            if "graph_topic" in config and config["graph_topic"]:
                if config["graph_topic"] in dashboard.available_topics:
                    dashboard.subscribe_to_graph_topic(config["graph_topic"])
                    # Update graph topic combobox in main UI
                    if hasattr(dashboard, 'graph_topic_combo'):
                        dashboard.graph_topic_combo.set(config["graph_topic"])
                    print(f"Loaded graph topic: {config['graph_topic']}")
            
            # Load PID settings for Robot 1
            if "pid_p1_value" in config and config["pid_p1_value"] is not None:
                dashboard.pid_p1_value = config["pid_p1_value"]
                if hasattr(dashboard, 'pid_p1_entry'):
                    dashboard.pid_p1_entry.delete(0, tk.END)  # Clear first
                    dashboard.pid_p1_entry.insert(0, str(dashboard.pid_p1_value))
            if "pid_i1_value" in config and config["pid_i1_value"] is not None:
                dashboard.pid_i1_value = config["pid_i1_value"]
                if hasattr(dashboard, 'pid_i1_entry'):
                    dashboard.pid_i1_entry.delete(0, tk.END)  # Clear first
                    dashboard.pid_i1_entry.insert(0, str(dashboard.pid_i1_value))
            if "pid_d1_value" in config and config["pid_d1_value"] is not None:
                dashboard.pid_d1_value = config["pid_d1_value"]
                if hasattr(dashboard, 'pid_d1_entry'):
                    dashboard.pid_d1_entry.delete(0, tk.END)  # Clear first
                    dashboard.pid_d1_entry.insert(0, str(dashboard.pid_d1_value))
            
            # Print Robot 1 PID settings if available
            if all(key in config for key in ["pid_p1_value", "pid_i1_value", "pid_d1_value"]):
                if config["pid_p1_value"] is not None:
                    print(f"Loaded Robot 1 PID settings: P={config['pid_p1_value']}, I={config['pid_i1_value']}, D={config['pid_d1_value']}")
            
            # Load PID settings for Robot 2
            if "pid_p2_value" in config and config["pid_p2_value"] is not None:
                dashboard.pid_p2_value = config["pid_p2_value"]
                if hasattr(dashboard, 'pid_p2_entry'):
                    dashboard.pid_p2_entry.delete(0, tk.END)  # Clear first
                    dashboard.pid_p2_entry.insert(0, str(dashboard.pid_p2_value))
            if "pid_i2_value" in config and config["pid_i2_value"] is not None:
                dashboard.pid_i2_value = config["pid_i2_value"]
                if hasattr(dashboard, 'pid_i2_entry'):
                    dashboard.pid_i2_entry.delete(0, tk.END)  # Clear first
                    dashboard.pid_i2_entry.insert(0, str(dashboard.pid_i2_value))
            if "pid_d2_value" in config and config["pid_d2_value"] is not None:
                dashboard.pid_d2_value = config["pid_d2_value"]
                if hasattr(dashboard, 'pid_d2_entry'):
                    dashboard.pid_d2_entry.delete(0, tk.END)  # Clear first
                    dashboard.pid_d2_entry.insert(0, str(dashboard.pid_d2_value))
            
            # Print Robot 2 PID settings if available
            if all(key in config for key in ["pid_p2_value", "pid_i2_value", "pid_d2_value"]):
                if config["pid_p2_value"] is not None:
                    print(f"Loaded Robot 2 PID settings: P={config['pid_p2_value']}, I={config['pid_i2_value']}, D={config['pid_d2_value']}")
            
            # Handle legacy config format - migrate old single-robot PID values to Robot 1
            if all(key in config for key in ["pid_p_value", "pid_i_value", "pid_d_value"]):
                if config["pid_p_value"] is not None and not hasattr(dashboard, "pid_p1_value"):
                    print("Migrating legacy PID settings to Robot 1...")
                    dashboard.pid_p1_value = config["pid_p_value"]
                    dashboard.pid_i1_value = config["pid_i_value"]
                    dashboard.pid_d1_value = config["pid_d_value"]
                    
                    # Update entries if they exist
                    if hasattr(dashboard, 'pid_p1_entry'):
                        dashboard.pid_p1_entry.delete(0, tk.END)
                        dashboard.pid_p1_entry.insert(0, str(dashboard.pid_p1_value))
                    if hasattr(dashboard, 'pid_i1_entry'):
                        dashboard.pid_i1_entry.delete(0, tk.END)
                        dashboard.pid_i1_entry.insert(0, str(dashboard.pid_i1_value))
                    if hasattr(dashboard, 'pid_d1_entry'):
                        dashboard.pid_d1_entry.delete(0, tk.END)
                        dashboard.pid_d1_entry.insert(0, str(dashboard.pid_d1_value))
        else:
            print(f"Configuration file {dashboard.config_file} does not exist. Using default settings.")
    
    except Exception as e:
        print(f"Error loading configuration: {e}")