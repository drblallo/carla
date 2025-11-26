#!/usr/bin/env python

"""
Convert CARLA simulation data (pickle) to JSON format compatible with V2X trajectory format.
Generates vehicle-N-M.json files where:
- N is the vehicle number (starting from 1)
- M is the Mth 10-second trajectory segment for that vehicle
"""

import pickle
import json
import os
import sys
from typing import Dict, List, Any
from datetime import datetime, timedelta
import argparse


def load_carla_data(pickle_file: str) -> Dict[str, Any]:
    """Load CARLA simulation data from pickle file."""
    with open(pickle_file, 'rb') as f:
        data = pickle.load(f)
    return data


def convert_speed_to_v2x_format(speed_ms: float) -> int:
    """
    Convert speed from m/s to V2X format (centimeters per second).
    
    Args:
        speed_ms: Speed in meters per second
        
    Returns:
        Speed in cm/s as integer
    """
    return int(speed_ms * 100)


def convert_heading_to_v2x_format(heading_deg: float) -> int:
    """
    Convert heading from degrees to V2X format (decidegrees).
    
    Args:
        heading_deg: Heading in degrees (0-360)
        
    Returns:
        Heading in decidegrees (0-3600) as integer
    """
    return int(heading_deg * 10)


def convert_altitude_to_v2x_format(altitude_m: float) -> int:
    """
    Convert altitude from meters to V2X format (decimeters).
    
    Args:
        altitude_m: Altitude in meters
        
    Returns:
        Altitude in decimeters as integer
    """
    return int(altitude_m * 10)


def create_trajectory_point(state: Dict[str, Any], station_id: int, position_type: str) -> Dict[str, Any]:
    """
    Create a single trajectory point in V2X JSON format.
    
    Args:
        state: CARLA trajectory state dictionary
        station_id: Vehicle station ID
        position_type: Either "position_GNSS" or "position_estimated"
        
    Returns:
        Trajectory point dictionary in V2X format
    """
    # Extract data from CARLA state
    geo_loc = state.get('geo_location', {})
    velocity = state.get('velocity', {})
    
    # Create the point structure
    point = {
        "stationInfo": {
            "stationId": station_id,
            "stationType": "passengerCar"  # Could be derived from type_id if needed
        },
        "objectId": None,
        "positionType": position_type,
        "time": state['timestamp'].isoformat() + 'Z',  # timestamp is already a datetime object
        "position": {
            "latitude": round(geo_loc.get('latitude', 0), 7),
            "longitude": round(geo_loc.get('longitude', 0), 7),
            "positionConfidence": None,
            "altitude": {
                "value": convert_altitude_to_v2x_format(geo_loc.get('altitude', 0)),
                "confidence": None
            }
        },
        "speed": {
            "value": convert_speed_to_v2x_format(velocity.get('speed', 0)),
            "confidence": None
        },
        "heading": {
            "value": convert_heading_to_v2x_format(state.get('heading', 0)),
            "confidence": None
        },
        "driveDirection": None
    }
    
    return point


def extract_10s_windows(trajectory: List[Dict[str, Any]], sample_frequency: int = 10) -> List[List[Dict[str, Any]]]:
    """
    Split trajectory into overlapping 10-second windows with 1-second stride.
    
    Args:
        trajectory: Full trajectory list
        sample_frequency: Sampling frequency in Hz
        
    Returns:
        List of trajectory windows, each containing exactly 100 points (10s at 10Hz)
    """
    points_per_window = sample_frequency * 10  # 10 seconds * 10 Hz = 100 points
    stride = sample_frequency * 1  # 1 second stride = 10 points
    windows = []
    
    # Create overlapping windows with 1-second stride
    for i in range(0, len(trajectory) - points_per_window + 1, stride):
        window = trajectory[i:i + points_per_window]
        # Only include complete windows (exactly 100 points)
        if len(window) == points_per_window:
            windows.append(window)
    
    return windows


def create_vehicle_json(vehicle_id: int, trajectory_window: List[Dict[str, Any]], 
                       station_id: int, sample_frequency: int = 10) -> Dict[str, Any]:
    """
    Create the complete JSON structure for a vehicle trajectory segment.
    
    Args:
        vehicle_id: CARLA vehicle ID
        trajectory_window: List of trajectory states for this 10s window
        station_id: V2X station ID for this vehicle
        sample_frequency: Sampling frequency in Hz
        
    Returns:
        Complete JSON structure
    """
    # Determine which points are GNSS vs estimated
    # Typically, every 10th point (every second) is GNSS, others are estimated
    points = []
    for idx, state in enumerate(trajectory_window):
        # Mark every 10th point as GNSS (at 1-second intervals)
        position_type = "position_GNSS" if idx % sample_frequency == 0 else "position_estimated"
        point = create_trajectory_point(state, station_id, position_type)
        points.append(point)
    
    # Reverse to match the example format (most recent first)
    points.reverse()
    
    vehicle_json = {
        "sampleFrequency": sample_frequency,
        "vehicleLength": None,  # Not available in CARLA data
        "vehicleWidth": None,   # Not available in CARLA data
        "points": points
    }
    
    return vehicle_json


def convert_carla_to_json(pickle_file: str, output_dir: str = "output_trajectories", 
                         sample_frequency: int = 10):
    """
    Main conversion function from CARLA pickle to V2X JSON format.
    
    Args:
        pickle_file: Path to CARLA pickle file
        output_dir: Directory to save JSON files
        sample_frequency: Expected sampling frequency in Hz
    """
    # Load CARLA data
    print(f"Loading CARLA data from {pickle_file}...")
    data = load_carla_data(pickle_file)
    
    # Extract trajectories
    trajectories = data.get('trajectories', {})
    vehicles = trajectories.get('vehicles', {})
    
    if not vehicles:
        print("No vehicle trajectories found in the data!")
        return
    
    print(f"Found {len(vehicles)} vehicles")
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Process each vehicle
    vehicle_counter = 1
    total_files = 0
    
    for carla_vehicle_id, trajectory in vehicles.items():
        if not trajectory:
            print(f"Skipping vehicle {carla_vehicle_id} - empty trajectory")
            continue
        
        print(f"Processing vehicle {carla_vehicle_id} ({vehicle_counter}): {len(trajectory)} points")
        
        # Split into 10-second windows
        windows = extract_10s_windows(trajectory, sample_frequency)
        
        if not windows:
            print(f"  No complete windows found for vehicle {carla_vehicle_id}")
            continue
        
        print(f"  Created {len(windows)} 10-second windows")
        
        # Use CARLA vehicle ID as station ID (or could use vehicle_counter)
        station_id = carla_vehicle_id
        
        # Create JSON for each window
        for window_idx, window in enumerate(windows, start=1):
            vehicle_json = create_vehicle_json(
                vehicle_id=carla_vehicle_id,
                trajectory_window=window,
                station_id=station_id,
                sample_frequency=sample_frequency
            )
            
            # Generate filename: vehicle-N-M.json
            filename = f"vehicle-{vehicle_counter}-{window_idx}.json"
            filepath = os.path.join(output_dir, filename)
            
            # Save JSON file
            with open(filepath, 'w') as f:
                json.dump(vehicle_json, f, indent=4)
            
            total_files += 1
            print(f"  Saved {filename} ({len(window)} points)")
        
        vehicle_counter += 1
    
    print(f"\nConversion complete!")
    print(f"Generated {total_files} JSON files in {output_dir}/")
    print(f"Processed {vehicle_counter - 1} vehicles")


def main():
    parser = argparse.ArgumentParser(
        description="Convert CARLA simulation pickle data to V2X JSON trajectory format"
    )
    parser.add_argument(
        "pickle_file",
        help="Path to CARLA pickle file (e.g., output/simulation_data.pkl)"
    )
    parser.add_argument(
        "-o", "--output-dir",
        default="output_trajectories",
        help="Output directory for JSON files (default: output_trajectories)"
    )
    parser.add_argument(
        "-f", "--frequency",
        type=int,
        default=10,
        help="Sampling frequency in Hz (default: 10)"
    )
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.pickle_file):
        print(f"Error: File '{args.pickle_file}' not found!")
        sys.exit(1)
    
    # Run conversion
    convert_carla_to_json(
        pickle_file=args.pickle_file,
        output_dir=args.output_dir,
        sample_frequency=args.frequency
    )


if __name__ == "__main__":
    main()

# python VFA_carla_scripts/carla_to_json.py /home/innovation/carla/PythonAPI/examples/output/scenario1_crash_data.pkl -o /home/innovation/carla/PythonAPI/examples/output/json
