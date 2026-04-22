import time
import yaml
from pathlib import Path

class TaskRecorder:
    def __init__(self):
        self.is_recording = False
        self.start_time = 0
        self.sequence = []

    def start(self):
        self.sequence = []
        self.start_time = time.time()
        self.is_recording = True

    def stop(self):
        self.is_recording = False

    def add_entry(self, entry_type, robot_name, data):
        if not self.is_recording:
            return
        
        timestamp = time.time() - self.start_time
        entry = {
            'timestamp': round(timestamp, 3),
            'type': entry_type,
            'robot': robot_name,
            'data': data
        }
        self.sequence.append(entry)

    def save_to_yaml(self, file_path):
        data = {
            'metadata': {
                'created_at': time.strftime("%Y-%m-%d %H:%M:%S"),
                'total_actions': len(self.sequence)
            },
            'sequence': self.sequence
        }
        with open(file_path, 'w') as f:
            yaml.dump(data, f, sort_keys=False)

class PlaybackEngine:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.is_playing = False
        self.sequence = []

    def load_task(self, file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            self.sequence = data.get('sequence', [])
        return self.sequence

    def execute(self, callback_func=None):
        if not self.sequence:
            return
        
        self.is_playing = True
        start_playback = time.time()
        
        for entry in self.sequence:
            if not self.is_playing:
                break
            
            # Simple wait-based timing (can be improved with QTimer)
            target_time = entry['timestamp']
            current_offset = time.time() - start_playback
            wait_time = target_time - current_offset
            
            if wait_time > 0:
                time.sleep(wait_time)
            
            # Execute action
            self._dispatch_action(entry)
            
            if callback_func:
                callback_func(entry)

        self.is_playing = False

    def _dispatch_action(self, entry):
        # Implementation of publishing/calling services based on entry type
        pass

    def stop(self):
        self.is_playing = False
