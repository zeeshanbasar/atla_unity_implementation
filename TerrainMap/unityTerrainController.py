import socket
import json
import numpy as np
# import time
# import matplotlib.pyplot as plt

class UnityTerrainController:
    def __init__(self, host='127.0.0.1', port=5555):
        self.host = host
        self.port = port
        self.data_path = r"C:\[[WORK]]\[[AERO]]\0. Thesis\CodeBase\8. OTBU Py Migration\TerrainMap\Assets\terrain_data.json"
    
    def send_command(self, command_dict):
        """Send a command to Unity and get response"""
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((self.host, self.port))
            
            # Send command
            message = json.dumps(command_dict)
            client.send(message.encode('utf-8'))
            
            # Receive response
            response = client.recv(1024).decode('utf-8')
            client.close()
            
            return json.loads(response)
        except Exception as e:
            print(f"Error sending command: {e}")
            return {"status": "error", "message": str(e)}
    
    def move_camera(self, x, y, z):
        """Move the camera to position (x, y, z)"""
        command = {
            "command": "move_camera",
            "x": x,
            "y": y,
            "z": z
        }
        response = self.send_command(command)
        # print(f"Move camera response: {response}")
        return response

    def move_and_export(self, x, y, z):
        """Move camera and export data in one operation"""
        command = {
            "command": "move_and_export",
            "x": x,
            "y": y,
            "z": z
        }
        response = self.send_command(command)
        # print(f"Move and export response: {response}")
        
        return response
    
    def load_terrain_data(self):
        """Load the exported terrain data in new schema format"""
        try:
            with open(self.data_path, 'r') as f:
                data = json.load(f)
            
            x = np.array(data['x'])  # 1D array of x coordinates
            y = np.array(data['y'])  # 1D array of y coordinates
            z = np.array(data['z'])  # 2D array of heights
            
            # print(f"Loaded terrain data:")
            # print(f"  x shape: {x.shape}")
            # print(f"  y shape: {y.shape}")
            # print(f"  z shape: {z.shape}")
            
            return {'x': x, 'y': y, 'z': z}
        except FileNotFoundError:
            print(f"Terrain data file not found at {self.data_path}")
            return None
