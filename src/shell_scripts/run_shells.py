import subprocess
import tkinter as tk
from tkinter import scrolledtext
import select

class ScriptRunner:
    def __init__(self, script_name, script_path):
        self.script_name = script_name
        self.script_path = script_path
        self.process = None
        self.window = None
        self.output_text = None

    def run_script(self):
        self.window = tk.Toplevel()
        self.window.title(self.script_name)

        self.output_text = scrolledtext.ScrolledText(self.window, width=60, height=10)
        self.output_text.pack(padx=10, pady=10)

        self.process = subprocess.Popen(['bash', self.script_path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)

        self.window.after(100, self.update_output)

    def update_output(self):
        self.output="a"
        if self.process.poll() is None:
            pass
        else:
            pass
        ready_to_read, _, _ = select.select([self.process.stdout], [], [], 0.05)
        if self.process.stdout in ready_to_read:
            output = self.process.stdout.readline()
            # if output.strip():
            self.output_text.insert(tk.END, output)
            self.output_text.see(tk.END)  # Scroll to the end
        else:
            # No new output available at the moment
            pass
            # Process has finished
            # self.output_text.insert(tk.END, "Script execution completed.")
            # self.output_text.see(tk.END)  # Scroll to the end
        self.window.after(100, self.update_output)
    def terminate_script(self):
        if self.process:
            self.process.terminate()
        self.window.destroy()

def create_script_buttons():
    script_buttons_frame = tk.Frame(root)
    script_buttons_frame.pack()

    script_runners = [
        ScriptRunner("Script 1", "/home/shadow1/zetaxbot/src/shell_scripts/cv.sh"),
        ScriptRunner("Script 2", "/home/shadow1/zetaxbot/src/shell_scripts/remote.sh"),
        ScriptRunner("Script 3", "/home/shadow1/zetaxbot/src/shell_scripts/complete.sh")
    ]

    for script_runner in script_runners:
        button_frame = tk.Frame(script_buttons_frame)
        button_frame.pack(side=tk.LEFT, padx=5, pady=5)

        # Create run button for the script
        run_button = tk.Button(button_frame, text=f"Run {script_runner.script_name}", command=script_runner.run_script)
        run_button.pack(side=tk.LEFT)

        # Create terminate button for the script
        terminate_button = tk.Button(button_frame, text=f"Terminate {script_runner.script_name}", command=script_runner.terminate_script)
        terminate_button.pack(side=tk.BOTTOM)

# Create GUI
root = tk.Tk()
root.title("Script Runner")
root.geometry("600x400")

# Create script buttons
create_script_buttons()

# Run the GUI event loop
root.mainloop()