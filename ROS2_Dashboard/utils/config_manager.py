import os
import json
import tkinter as tk
import datetime

def save_config(dashboard):
    """Save configuration to a JSON file"""
    # Create configuration dictionary with only necessary items
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
        
        "PID last update": getattr(dashboard, "pid_last_update", None),
    }
    
    # Add PID settings for both robots and all axes
    for robot_num in [1, 2]:
        for axis in ['x', 'y', 'yaw']:
            p_attr = f"pid_p{robot_num}_{axis}_value"
            i_attr = f"pid_i{robot_num}_{axis}_value"
            d_attr = f"pid_d{robot_num}_{axis}_value"
            
            config[p_attr] = getattr(dashboard, p_attr, None)
            config[i_attr] = getattr(dashboard, i_attr, None)
            config[d_attr] = getattr(dashboard, d_attr, None)
    
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
        
        # Print PID settings for both robots and all axes
        for robot_num in [1, 2]:
            for axis in ['x', 'y', 'yaw']:
                p_attr = f"pid_p{robot_num}_{axis}_value"
                i_attr = f"pid_i{robot_num}_{axis}_value"
                d_attr = f"pid_d{robot_num}_{axis}_value"
                
                if all(key in clean_config for key in [p_attr, i_attr, d_attr]):
                    print(f"Robot {robot_num} {axis.upper()} PID settings saved: P={clean_config[p_attr]}, I={clean_config[i_attr]}, D={clean_config[d_attr]}")
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
            for i in range(1, 7):
                topic_key = f"topic{i}"
                if topic_key in config and config[topic_key]:
                    if config[topic_key] in dashboard.available_topics:
                        dashboard.subscribe_to_topic(i, config[topic_key])
                        # Also update combobox in main UI
                        if hasattr(dashboard, f"topic{i}_combo"):
                            getattr(dashboard, f"topic{i}_combo").set(config[topic_key])
            
            # Load position topics
            for i in range(1, 3):
                pos_key = f"pos_topic{i}"
                if pos_key in config and config[pos_key]:
                    if config[pos_key] in dashboard.available_topics:
                        dashboard.subscribe_to_position(config[pos_key], i)
            
            # Load output topic
            if "output_topic" in config and config["output_topic"]:
                dashboard.output_topic = config["output_topic"]
                if hasattr(dashboard, "output_topics"):
                    dashboard.output_topics.set(dashboard.output_topic)
            
            # Load graph topic
            if "graph_topic" in config and config["graph_topic"]:
                if config["graph_topic"] in dashboard.available_topics:
                    dashboard.subscribe_to_graph_topic(config["graph_topic"])
                    if hasattr(dashboard, 'graph_topic_combo'):
                        dashboard.graph_topic_combo.set(config["graph_topic"])
                        
            # Load PID last update
            if "PID last update" in config and config["PID last update"]:
                dashboard.pid_last_update = config["PID last update"]
            
            # Load PID settings for both robots and all axes
            for robot_num in [1, 2]:
                for axis in ['x', 'y', 'yaw']:
                    p_attr = f"pid_p{robot_num}_{axis}_value"
                    i_attr = f"pid_i{robot_num}_{axis}_value"
                    d_attr = f"pid_d{robot_num}_{axis}_value"
                    
                    if p_attr in config and config[p_attr] is not None:
                        setattr(dashboard, p_attr, config[p_attr])
                    if i_attr in config and config[i_attr] is not None:
                        setattr(dashboard, i_attr, config[i_attr])
                    if d_attr in config and config[d_attr] is not None:
                        setattr(dashboard, d_attr, config[d_attr])
                    
                    # Print if all values are available
                    if all(key in config for key in [p_attr, i_attr, d_attr]):
                        print(f"Loaded Robot {robot_num} {axis.upper()} PID settings: P={config[p_attr]}, I={config[i_attr]}, D={config[d_attr]}")
            
            # Handle migration of old PID format
            migrate_legacy_pid_settings(dashboard, config)
        else:
            print(f"Configuration file {dashboard.config_file} does not exist. Using default settings.")
    
    except Exception as e:
        print(f"Error loading configuration: {e}")

def migrate_legacy_pid_settings(dashboard, config):
    """Migrate legacy PID settings to the new format"""
    # Migrate old non-axis specific PID values to x-axis
    for robot_num in [1, 2]:
        old_p = f"pid_p{robot_num}_value"
        old_i = f"pid_i{robot_num}_value"
        old_d = f"pid_d{robot_num}_value"
        
        new_p = f"pid_p{robot_num}_x_value"
        new_i = f"pid_i{robot_num}_x_value"
        new_d = f"pid_d{robot_num}_x_value"
        
        if all(key in config for key in [old_p, old_i, old_d]) and not hasattr(dashboard, new_p):
            print(f"Migrating legacy PID settings for Robot {robot_num} to X-axis...")
            setattr(dashboard, new_p, config[old_p])
            setattr(dashboard, new_i, config[old_i])
            setattr(dashboard, new_d, config[old_d])
    
    # Migrate very old single-robot PID values
    if all(key in config for key in ["pid_p_value", "pid_i_value", "pid_d_value"]):
        if not hasattr(dashboard, "pid_p1_x_value"):
            print("Migrating very old legacy PID settings to Robot 1 X-axis...")
            dashboard.pid_p1_x_value = config["pid_p_value"]
            dashboard.pid_i1_x_value = config["pid_i_value"]
            dashboard.pid_d1_x_value = config["pid_d_value"]