#!/usr/bin/env python3
"""
Enhanced SUMO server with traffic light control and induction loop detection.
Automatically reloads simulation when clients disconnect.
"""

import traci
import json
import socket
import argparse
import time
import threading
from typing import List, Dict

HOST = "127.0.0.1"
PORT = 55000

# Traffic light state tracking
traffic_light_states = {}

def collect_vehicle_data(veh_ids: List[str]) -> Dict[str, Dict]:
    """Collect position, speed, and lane for each vehicle."""
    data = {}
    for vid in veh_ids:
        try:
            pos = traci.vehicle.getPosition(vid)
            speed = traci.vehicle.getSpeed(vid)
            lane = traci.vehicle.getLaneID(vid)
            data[vid] = {
                "pos": [float(pos[0]), float(pos[1])],
                "speed": float(speed),
                "lane": lane
            }
        except traci.TraCIException:
            continue
    return data

def collect_detector_data(detector_ids: List[str]) -> Dict[str, bool]:
    """Collect induction loop detector states."""
    data = {}
    for det_id in detector_ids:
        try:
            # Check if any vehicle is on the detector
            vehicles_on_detector = traci.inductionloop.getLastStepVehicleNumber(det_id)
            data[det_id] = 1 if vehicles_on_detector > 0 else 0
        except traci.TraCIException:
            data[det_id] = 0
            print(f"ERROR AT DETECTOR {det_id}")
    return data

def collect_area_detector_data(area_detector_ids: List[str]) -> Dict[str, Dict]:
    """Collect area detector data (vehicle count and mean speed)."""
    data = {}
    for det_id in area_detector_ids:
        try:
            vehicle_count = traci.lanearea.getLastStepVehicleNumber(det_id)
            mean_speed = traci.lanearea.getLastStepMeanSpeed(det_id)
            data[det_id] = {
                "vehicle_count": int(vehicle_count),
                "mean_speed": float(mean_speed)
            }
        except traci.TraCIException:
            data[det_id] = {
                "vehicle_count": 0,
                "mean_speed": 0.0
            }
            print(f"ERROR AT AREA DETECTOR {det_id}")
    return data

def initialize_traffic_lights():
    """Initialize traffic light control and get their IDs."""
    global traffic_light_states
    
    # Get all traffic light IDs
    tl_ids = traci.trafficlight.getIDList()
    print(f"Found traffic lights: {tl_ids}")
    
    # Initialize state tracking for each traffic light
    for tl_id in tl_ids:
        traffic_light_states[tl_id] = {
            'current_phase': traci.trafficlight.getPhase(tl_id),
            'target_phase': None,
            'transitioning': False,
            'manual_control': True
        }
        
        # Set to manual control mode (stop automatic phase progression)
        try:
            traci.trafficlight.setPhase(tl_id, traci.trafficlight.getPhase(tl_id))
        except:
            pass
    
    return tl_ids

def get_transition_sequence(current_phase: int, target_phase: int, program_logic: List) -> List[int]:
    """
    Get the sequence of phases to transition from current to target phase.
    This is a simplified version - you may need to adjust based on your traffic light program.
    """
    if current_phase == target_phase:
        return []
    
    # Simple transition logic - you may need to customize this based on your traffic light phases
    # For many traffic lights: phase 0,1 = green for one direction, phase 2,3 = yellow, phase 4,5 = green for other direction, etc.
    
    if target_phase == 2:  # Want to go to phase 2
        if current_phase in [0, 1]:  # Currently green for direction 1
            return [2]  # Go directly to phase 2
        elif current_phase in [4, 5]:  # Currently green for direction 2
            return [6, 2] if 6 < len(program_logic) else [2]  # Transition through yellow if exists
        else:
            return [2]
    
    elif target_phase == 4:  # Want to go to phase 4
        if current_phase in [4, 5]:  # Already in direction 2
            return [4]
        elif current_phase in [0, 1, 2]:  # Currently in direction 1
            return [3, 4] if 3 < len(program_logic) else [4]  # Transition through yellow if exists
        else:
            return [4]
    
    return [target_phase]

def handle_traffic_light_command(command: Dict):
    """Handle traffic light control commands from MATLAB."""
    global traffic_light_states
    
    try:
        if command.get('type') == 'trafficlight':
            junction = command.get('junction')  # Specific junction ID
            target_phase = int(command.get('phase'))  # Direct phase number
            
            # If no specific junction specified, apply to all traffic lights
            target_junctions = [junction] if junction else list(traffic_light_states.keys())
            
            for tl_id in target_junctions:
                if tl_id in traffic_light_states:
                    # Set phase directly without transition
                    try:
                        traci.trafficlight.setPhase(tl_id, target_phase)
                        traffic_light_states[tl_id]['current_phase'] = target_phase
                        traffic_light_states[tl_id]['target_phase'] = None
                        traffic_light_states[tl_id]['transitioning'] = False
                        print(f"Traffic light {tl_id}: set directly to phase {target_phase}")
                    except Exception as e:
                        print(f"Error setting traffic light {tl_id} to phase {target_phase}: {e}")
                else:
                    print(f"Unknown traffic light ID: {tl_id}")
                    
    except Exception as e:
        print(f"Error handling traffic light command: {e}")

def update_traffic_lights():
    """Update traffic light phases - simplified since we're setting phases directly."""
    global traffic_light_states
    
    for tl_id, state in traffic_light_states.items():
        try:
            # Just update current phase tracking
            current_phase = traci.trafficlight.getPhase(tl_id)
            state['current_phase'] = current_phase
        except Exception as e:
            print(f"Error reading traffic light {tl_id}: {e}")

def collect_instant_induction_loop_data(instant_loop_ids: List[str]) -> Dict[str, int]:
    """Collect instant induction loop data (pedestrian count)."""
    data = {}
    for loop_id in instant_loop_ids:
        try:
            # Get the number of vehicles/pedestrians that passed the instant loop
            # in the last simulation step
            vehicle_count = traci.inductionloop.getLastStepVehicleNumber(loop_id)
            data[loop_id] = int(vehicle_count)
        except traci.TraCIException as e:
            data[loop_id] = 0
            print(f"ERROR AT INSTANT INDUCTION LOOP {loop_id}: {e}")
    return data

def listen_for_commands(conn, stop_event):
    """Listen for commands from MATLAB client in a separate thread."""
    try:
        conn.settimeout(1.0)
        command_count = 0
        
        while not stop_event.is_set():
            try:
                raw_command = conn.recv(1024).decode('utf-8').strip()
                
                if not raw_command:
                    print("DEBUG: Empty command received (connection may be closing)")
                    break
                
                command_count += 1
                print(f"[CMD #{command_count}] Received: {raw_command}")
                
                command = json.loads(raw_command)
                handle_traffic_light_command(command)
                
            except socket.timeout:
                # This is expected - no data available
                continue
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e} - Raw: {raw_command}")
                continue  # Don't exit, keep listening
            except ConnectionResetError:
                print("Connection reset by peer - command thread exiting")
                break
            except OSError as e:
                print(f"OS Error in command thread: {e}")
                break
            except Exception as e:
                print(f"Unexpected error in command thread: {type(e).__name__}: {e}")
                continue  # Don't exit on unexpected errors
                
    except Exception as e:
        print(f"CRITICAL: Command listener setup error: {e}")
    finally:
        print("Command listening thread has stopped")

def reset_simulation(sumocfg: str, gui: bool):
    """Reset SUMO simulation to initial state."""
    global traffic_light_states
    
    try:
        traci.close()
        print("Closed current simulation")
    except:
        pass
    
    # Start new simulation
    sumo_binary = "sumo-gui" if gui else "sumo"
    sumo_cmd = [sumo_binary, "-c", sumocfg, "--start"]
    traci.start(sumo_cmd)
    
    # Reinitialize traffic lights
    initialize_traffic_lights()
    print("SUMO simulation reset to initial state")

def handle_client_session(conn, addr, sumocfg: str, steps: int, gui: bool):
    """Handle a single client session with area detector data and instant induction loops."""
    conn_file = conn.makefile("w")
    print(f"Client connected from {addr}")
    
    detector_ids = [f"e1_{i}" for i in range(4)]
    instant_loop_ids = [f"e1i_{i}" for i in range(8)]  # New instant loops
    area_detector_ids = ["e2_0", "e2_1", "e2_2", "e2_3", "e2_4", "e2_5", "e2_6", "e2_7", "e2_8", "e2_9", "e2_10", "e2_11", "e2_12", "e2_13", "e2_14", "e2_15", "e2_16", "e2_17"]
    
    stop_event = threading.Event()
    
    command_thread = threading.Thread(
        target=listen_for_commands, 
        args=(conn, stop_event), 
        daemon=False
    )
    command_thread.start()
    print("Command listener thread started")
    
    client_disconnected = False
    send_errors = 0
    max_send_errors = 5
    
    try:
        for step in range(steps):
            traci.simulationStep()
            update_traffic_lights()
            
            t = traci.simulation.getTime()
            veh_ids = traci.vehicle.getIDList()
            vehicles = collect_vehicle_data(veh_ids)
            detectors = collect_detector_data(detector_ids)
            instant_loops = collect_instant_induction_loop_data(instant_loop_ids)  # New
            area_detectors = collect_area_detector_data(area_detector_ids)

            payload = {
                "time": float(t),
                "step": step,
                "n_vehicles": len(vehicles),
                "vehicles": vehicles,
                "detectors": detectors,
                "instant_loops": instant_loops,  # New
                "area_detectors": area_detectors,
                "traffic_lights": {tl_id: state['current_phase'] for tl_id, state in traffic_light_states.items()},
            }

            time.sleep(0.1)

            try:
                conn_file.write(json.dumps(payload) + "\n")
                conn_file.flush()
                send_errors = 0
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                print(f"Send error ({send_errors+1}/{max_send_errors}): {e}")
                send_errors += 1
                if send_errors >= max_send_errors:
                    print("Too many send errors - client disconnected")
                    client_disconnected = True
                    break
                continue

            if step % 100 == 0:
                area_vehicles = sum([data["vehicle_count"] for data in area_detectors.values()])
                pedestrians = sum(instant_loops.values())  # New
                print(f"[t={t:.2f}] sent {len(vehicles)} vehicles, {sum(detectors.values())} active detectors, {area_vehicles} area detector vehicles, {pedestrians} pedestrians detected")
                
    except Exception as e:
        print(f"Error during simulation: {e}")
        client_disconnected = True
    finally:
        stop_event.set()
        command_thread.join(timeout=2.0)
        
        try:
            conn_file.close()
            conn.close()
        except:
            pass
        
        print("Session ended - client disconnected" if client_disconnected else "Session completed successfully")

def run_server(sumocfg: str, steps: int, step_length: float, gui: bool, port: int):
    """Main server loop that handles multiple client connections."""
    
    # Initial SUMO startup
    sumo_binary = "sumo-gui" if gui else "sumo"
    sumo_cmd = [sumo_binary, "-c", sumocfg, "--start"]
    print(f"Starting SUMO...")
    traci.start(sumo_cmd)
    print(f"SUMO started ({'GUI' if gui else 'CLI'})")
    
    # Initialize traffic lights
    initialize_traffic_lights()
    
    # Setup TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, port))
    server.listen(1)
    print(f"Server listening on port {port}")
    
    try:
        while True:
            print(f"Waiting for MATLAB/Simulink client...")
            
            try:
                conn, addr = server.accept()
                handle_client_session(conn, addr, sumocfg, steps, gui)
                print("Resetting simulation for next client...")
                reset_simulation(sumocfg, gui)
                
            except KeyboardInterrupt:
                print("\nServer shutdown requested")
                break
            except Exception as e:
                print(f"Error handling client: {e} - continuing server operation")
                try:
                    reset_simulation(sumocfg, gui)
                except Exception as reset_error:
                    print(f"Error resetting simulation: {reset_error}")
                    break
                
    finally:
        print("Shutting down server...")
        try:
            server.close()
            traci.close()
        except:
            pass
        print("Server closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sumocfg", default="map/main.sumocfg")
    parser.add_argument("--steps", type=int, default=10000)
    parser.add_argument("--step-length", type=float, default=0.1)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()
    
    print("Starting automated SUMO server...")
    print("Press Ctrl+C to stop the server")
    run_server(args.sumocfg, args.steps, args.step_length, True, args.port)