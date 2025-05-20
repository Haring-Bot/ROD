#!/usr/bin/env python3
# filepath: /home/fhtw_user/catkin_ws/src/ROD/completeStation/scripts/menu.py
import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))

def run_simple_service():
    try:
        messagebox.showinfo("Starting Process", "Einfachen Service gestartet.")
        subprocess.Popen(["python", os.path.join(current_dir, "test.py"), "1"])
    except Exception as e:
        messagebox.showerror("Error", f"Failed to run script: {str(e)}")

def run_advanced_service():
    try:
        messagebox.showinfo("Starting Process", "Erweiterten Service gestartet.")
        subprocess.Popen(["python", os.path.join(current_dir, "test.py"), "2"])
    except Exception as e:
        messagebox.showerror("Error", f"Failed to run script: {str(e)}")

def run_home_position():
    try:
        messagebox.showinfo("Starting Process", "RESET.")
        subprocess.Popen(["python", os.path.join(current_dir, "test.py"), "3"])
    except Exception as e:
        messagebox.showerror("Error", f"Failed to run script: {str(e)}")

#main window
root = tk.Tk()
root.title("ROD Robot Control Panel")
root.geometry("600x450")
root.configure(bg="#f0f0f0")


try:
    root.iconbitmap(os.path.join(current_dir, "robot_icon.ico"))
except:
    pass


header_frame = tk.Frame(root, bg="#2c3e50", padx=10, pady=10)
header_frame.pack(fill="x")


title_label = tk.Label(header_frame, 
                     text="Robot Station Control System",
                     font=("Arial", 16, "bold"),
                     bg="#2c3e50",
                     fg="white")
title_label.pack(pady=5)

description_label = tk.Label(header_frame, 
                           text="...",
                           font=("Arial", 10),
                           bg="#2c3e50",
                           fg="#ecf0f1")
description_label.pack(pady=5)

#main content frame
content_frame = tk.Frame(root, bg="#f0f0f0", padx=20, pady=20)
content_frame.pack(fill="both", expand=True)

#button
style = ttk.Style()
style.configure("TButton", font=("Arial", 12), padding=10)
style.configure("Big.TButton", font=("Arial", 14, "bold"), padding=15)

#button descriptions
simple_service_description = """
Einfacher Service:
- kurze Polierung
- grobes reinigen des Bildschirms
                $ 
"""

advanced_service_description = """
Advanced service mode:
- aufwändige Poliserung
- vollständige Bildschirmreinigung
             $$
"""

run_home_description = """
Reset aller Achsen auf Home Position für Wartungszwecke
"""

#Button 1
simple_frame = ttk.LabelFrame(content_frame, text="Einfacher Service", padding=10)
simple_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

desc_label1 = tk.Label(simple_frame, text=simple_service_description, justify=tk.LEFT, 
                      bg="#f0f0f0", font=("Arial", 10))
desc_label1.pack(pady=5, fill="x")

button1 = ttk.Button(simple_frame, text="Einfachen Service starten", command=run_simple_service, style="Big.TButton")
button1.pack(pady=10, fill="x")

#Button 2
advanced_frame = ttk.LabelFrame(content_frame, text="Erweiterter Service", padding=10)
advanced_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

desc_label2 = tk.Label(advanced_frame, text=advanced_service_description, justify=tk.LEFT,
                      bg="#f0f0f0", font=("Arial", 10))
desc_label2.pack(pady=5, fill="x")

button2 = ttk.Button(advanced_frame, text="Erweiterten Service starten", command=run_advanced_service, style="Big.TButton")
button2.pack(pady=10, fill="x")

#Button 3
home_frame = ttk.LabelFrame(content_frame, text="Home Position", padding=10)
home_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

desc_label2 = tk.Label(home_frame, text=run_home_description, justify=tk.LEFT,
                      bg="#f0f0f0", font=("Arial", 10))
desc_label2.pack(pady=5, fill="x")

button2 = ttk.Button(home_frame, text="RESET", command=run_home_position, style="TButton")
button2.pack(pady=10, fill="x")

#Button 3
home_frame = ttk.LabelFrame(content_frame, text="Home Position", padding=10)
home_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

desc_label3 = tk.Label(home_frame, text=run_home_description, justify=tk.LEFT,
                      bg="#f0f0f0", font=("Arial", 10))
desc_label3.pack(pady=5, fill="x")

button3 = ttk.Button(home_frame, text="RESET", command=run_home_position, style="TButton")
button3.pack(pady=10, fill="x")


content_frame.columnconfigure(0, weight=1)
content_frame.columnconfigure(1, weight=1)
content_frame.rowconfigure(0, weight=1)
content_frame.rowconfigure(1, weight=1)


status_frame = tk.Frame(root, bg="#34495e", padx=5, pady=5)
status_frame.pack(fill="x", side="bottom")

status_label = tk.Label(status_frame, 
                      text="Ready - ROD Robot Control System v1.0",
                      font=("Arial", 9),
                      bg="#34495e",
                      fg="white")
status_label.pack(side="left")


root.mainloop()
