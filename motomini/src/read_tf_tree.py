import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tkinter as tk
from tkinter import filedialog, messagebox
import threading

class TfViewerNode(Node):
    def __init__(self):
        super().__init__('tf_gui_node')
        # Buffer stores the transform tree
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

class TfApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("ROS 2 TF Tree & Transform Viewer")
        
        # 1. TF Tree Display Area
        tk.Label(root, text="TF Tree Configuration (YAML):").grid(row=0, column=0, sticky='w', padx=5)
        self.tree_text = tk.Text(root, height=10, width=60)
        self.tree_text.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        
        tk.Button(root, text="Refresh TF Tree", command=self.refresh_tree).grid(row=2, column=0, columnspan=2, pady=5)
        
        # 2. Frame Transform Inputs
        tk.Label(root, text="Target Frame (e.g., odom):").grid(row=3, column=0, sticky='e', padx=5)
        self.target_entry = tk.Entry(root, width=25)
        self.target_entry.grid(row=3, column=1, sticky='w', padx=5)
        
        tk.Label(root, text="Source Frame (e.g., base_link):").grid(row=4, column=0, sticky='e', padx=5)
        self.source_entry = tk.Entry(root, width=25)
        self.source_entry.grid(row=4, column=1, sticky='w', padx=5)
        
        tk.Button(root, text="Get Transform Origin", command=self.get_transform).grid(row=5, column=0, columnspan=2, pady=5)
        
        # 3. Transform Result Display
        tk.Label(root, text="Transform Result:").grid(row=6, column=0, sticky='w', padx=5)
        self.result_text = tk.Text(root, height=8, width=60)
        self.result_text.grid(row=7, column=0, columnspan=2, padx=5, pady=5)
        
        tk.Button(root, text="Save Result to File", command=self.save_result).grid(row=8, column=0, columnspan=2, pady=10)

    def refresh_tree(self):
        self.tree_text.delete(1.0, tk.END)
        # Pulls the entire known TF tree as a YAML formatted string
        yaml_data = self.node.tf_buffer.all_frames_as_yaml()
        if not yaml_data:
            yaml_data = "No TF data available yet. Ensure publishers are running."
        self.tree_text.insert(tk.END, yaml_data)

    def get_transform(self):
        target = self.target_entry.get().strip()
        source = self.source_entry.get().strip()
        
        if not target or not source:
            messagebox.showwarning("Input Error", "Please provide both Target and Source frames.")
            return
            
        try:
            # Look up the latest available transform
            trans = self.node.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            
            t = trans.transform.translation
            r = trans.transform.rotation
            
            res_str = f"Transform from '{source}' to '{target}':\n"
            res_str += f"Translation (Origin): x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}\n"
            res_str += f"Rotation (Quaternion): x={r.x:.4f}, y={r.y:.4f}, z={r.z:.4f}, w={r.w:.4f}\n"
            
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END, res_str)
            
        except TransformException as ex:
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END, f"Could not compute transform:\n{ex}")

    def save_result(self):
        result_data = self.result_text.get(1.0, tk.END).strip()
        if not result_data:
            messagebox.showinfo("Empty", "No result to save. Please generate a transform first.")
            return
            
        # Open a file dialog to save the output
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            title="Save Transform Data",
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
        )
        if filepath:
            with open(filepath, 'w') as f:
                f.write(result_data)
            messagebox.showinfo("Success", f"Data saved successfully to:\n{filepath}")

def main():
    rclpy.init()
    node = TfViewerNode()
    
    # Run the ROS 2 event loop in a daemon thread so tkinter can run in the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Initialize and start the GUI
    root = tk.Tk()
    app = TfApp(root, node)
    root.mainloop()
    
    # Clean up after closing the GUI window
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()