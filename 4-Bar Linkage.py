import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class RobotEngineeringLinkageApp:
    def __init__(self, root):
        self.root = root
        self.root.title("4-Bar Linkage Simulator Pro - LEE SUCHEOL")
        
        self.font_large = ('Arial', 18)
        self.font_bold = ('Arial', 18, 'bold')
        self.is_dragging = False
        
        # --- Left Panel ---
        self.left_panel = tk.Frame(root, padx=30, pady=30)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y)

        tk.Label(self.left_panel, text="Link Lengths", font=self.font_bold).pack(pady=10)
        self.entries = {}
        labels = [("Fixed Link (L1)", "10"), ("Crank (L2)", "4"), 
                  ("Coupler (L3)", "8"), ("Rocker (L4)", "6")]
        for text, default in labels:
            tk.Label(self.left_panel, text=text, font=self.font_large).pack()
            entry = tk.Entry(self.left_panel, font=self.font_large, width=10)
            entry.insert(0, default)
            entry.pack(pady=2)
            self.entries[text] = entry

        tk.Label(self.left_panel, text="Graph Size (Zoom)", font=self.font_large).pack(pady=5)
        self.size_entry = tk.Entry(self.left_panel, font=self.font_large, width=10)
        self.size_entry.insert(0, "20")
        self.size_entry.pack(pady=2)

        self.apply_btn = tk.Button(self.left_panel, text="Apply Changes", font=self.font_bold, 
                                   bg="#0078d7", fg="white", command=self.update_geometry_limits)
        self.apply_btn.pack(pady=15, fill=tk.X)

        # Input Angle & Slider
        tk.Label(self.left_panel, text="Input Angle (θ2)", font=self.font_bold).pack(pady=5)
        self.angle_var = tk.DoubleVar(value=60)
        self.angle_entry = tk.Entry(self.left_panel, font=self.font_large, width=10, textvariable=self.angle_var)
        self.angle_entry.pack()
        
        self.slider = tk.Scale(self.left_panel, from_=-360, to=360, orient=tk.HORIZONTAL, 
                               variable=self.angle_var, font=self.font_large, length=300,
                               command=self.on_slider_move)
        self.slider.pack(pady=10)

        # Settings
        self.elbow_var = tk.StringVar(value="Up")
        tk.Radiobutton(self.left_panel, text="Elbow Up", variable=self.elbow_var, value="Up", font=self.font_large, command=self.render_plot).pack(anchor=tk.W)
        tk.Radiobutton(self.left_panel, text="Elbow Down", variable=self.elbow_var, value="Down", font=self.font_large, command=self.render_plot).pack(anchor=tk.W)

        self.ic_var = tk.BooleanVar(value=False)
        tk.Checkbutton(self.left_panel, text="Show Inst. Centers", variable=self.ic_var, font=self.font_large, command=self.render_plot).pack(anchor=tk.W, pady=10)

        self.mode_var = tk.StringVar(value="Slider")
        tk.Radiobutton(self.left_panel, text="Slider Mode", variable=self.mode_var, value="Slider", font=self.font_large, command=self.toggle_mode).pack(anchor=tk.W)
        tk.Radiobutton(self.left_panel, text="Mouse Drag Mode", variable=self.mode_var, value="Mouse", font=self.font_large, command=self.toggle_mode).pack(anchor=tk.W)

        # --- Right Panel ---
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)

        self.dead_points = None
        self.g_status = ""
        self.update_geometry_limits()

    def on_slider_move(self, event):
        if self.dead_points:
            val = self.angle_var.get()
            if val < self.dead_points[0]: self.angle_var.set(self.dead_points[0])
            elif val > self.dead_points[1]: self.angle_var.set(self.dead_points[1])
        self.render_plot()

    def line_intersection(self, p1, p2, p3, p4):
        x1, y1 = p1; x2, y2 = p2
        x3, y3 = p3; x4, y4 = p4
        denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
        if abs(denom) < 1e-9: return None
        ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
        return (x1 + ua*(x2-x1), y1 + ua*(y2-y1))

    def update_geometry_limits(self):
        try:
            l1, l2, l3, l4 = [float(self.entries[k].get()) for k in ["Fixed Link (L1)", "Crank (L2)", "Coupler (L3)", "Rocker (L4)"]]
            links = sorted([l1, l2, l3, l4])
            s, l, p, q = links[0], links[-1], links[1], links[2]
            is_grashof = (s + l) <= (p + q)
            
            if is_grashof:
                if l2 == s: self.g_status = "Grashof: Crank-Rocker"
                elif l1 == s: self.g_status = "Grashof: Double-Crank"
                elif l3 == s: self.g_status = "Grashof: Drag-Link"
                else: self.g_status = "Grashof: Double-Rocker"
            else: self.g_status = "Non-Grashof: Double-Rocker"

            cos_limit = ( (l3+l4)**2 - l1**2 - l2**2 ) / (-2 * l1 * l2)
            self.slider.config(from_=-360, to=360); self.dead_points = None
            if not (is_grashof and (l2 == s or l1 == s)):
                if -1 <= cos_limit <= 1:
                    limit_angle = np.degrees(np.arccos(cos_limit))
                    self.dead_points = (-limit_angle, limit_angle)
                    self.slider.config(from_=-limit_angle, to=limit_angle)
            self.render_plot()
        except: pass

    def render_plot(self):
        try:
            l1, l2, l3, l4 = [float(self.entries[k].get()) for k in ["Fixed Link (L1)", "Crank (L2)", "Coupler (L3)", "Rocker (L4)"]]
            user_val = float(self.size_entry.get())
            limit = 400 / max(user_val, 0.1)
            
            theta2 = np.radians(self.angle_var.get())
            Ax, Ay = 0, 0
            Dx, Dy = l1, 0
            Bx, By = l2 * np.cos(theta2), l2 * np.sin(theta2)
            dist_BD = np.sqrt((Dx - Bx)**2 + (Dy - By)**2)
            cos_gamma = (l3**2 + dist_BD**2 - l4**2) / (2 * l3 * dist_BD)
            
            self.ax.clear()
            self.ax.set_xlim(-limit/2, limit); self.ax.set_ylim(-limit/2, limit); self.ax.grid(True, alpha=0.3)
            
            status_txt = f"Condition: {self.g_status}"
            if self.dead_points:
                status_txt += f"\nDead Point (-): {self.dead_points[0]:.2f}°\nDead Point (+): {self.dead_points[1]:.2f}°"
            self.ax.text(-limit*0.45, limit*0.95, status_txt, fontsize=16, color='darkred', fontweight='bold', va='top', bbox=dict(facecolor='white', alpha=0.8))

            if abs(cos_gamma) <= 1:
                gamma = np.arccos(cos_gamma)
                if self.elbow_var.get() == "Down": gamma = -gamma
                phi = np.arctan2(By - Dy, Bx - Dx)
                Cx = Bx + l3 * np.cos(phi + np.pi - gamma)
                Cy = By + l3 * np.sin(phi + np.pi - gamma)
                theta4_deg = np.degrees(np.arctan2(Cy - Dy, Cx - Dx))

                # Linkage Plot with Labels for Legend
                self.ax.plot([Ax, Dx], [Ay, Dy], 'k', lw=5, label='Fixed (L1)')
                self.ax.plot([Ax, Bx], [Ay, By], 'r', lw=5, label='Input (L2)')
                self.ax.plot([Bx, Cx], [By, Cy], 'b', lw=5, label='Coupler (L3)')
                self.ax.plot([Cx, Dx], [Cy, Dy], 'g', lw=5, label='Rocker (L4)')
                self.ax.scatter([Ax, Bx, Cx, Dx], [Ay, By, Cy, Dy], color='black', s=100, zorder=5)

                # Constant Output Angle Display
                self.ax.text(-limit*0.45, -limit*0.4, f"Output Angle (θ4): {theta4_deg:.2f}°", 
                             fontsize=22, color='blue', fontweight='bold', bbox=dict(facecolor='white', alpha=0.8))

                if self.ic_var.get():
                    I12, I23, I34, I14 = (Ax, Ay), (Bx, By), (Cx, Cy), (Dx, Dy)
                    I13 = self.line_intersection(I12, I14, I23, I34)
                    I24 = self.line_intersection(I12, I23, I14, I34)
                    ics = {"I12": I12, "I23": I23, "I34": I34, "I14": I14, "I13": I13, "I24": I24}
                    ic_coords_txt = "IC Coordinates (Ref: Ax,Ay)\n"
                    
                    for name, pos in ics.items():
                        if pos:
                            self.ax.scatter(pos[0], pos[1], color='purple', s=80, marker='x', zorder=6)
                            self.ax.text(pos[0], pos[1], f" {name}", color='purple', fontsize=12, fontweight='bold')
                            ic_coords_txt += f"{name}: ({pos[0]:.2f}, {pos[1]:.2f})\n"
                    
                    # dash-dot-dot styling
                    ls_style = (0, (3, 1, 1, 1, 1, 1))
                    if I13:
                        self.ax.plot([I12[0], I13[0]], [I12[1], I13[1]], color='gray', ls=ls_style, lw=1)
                        self.ax.plot([I23[0], I13[0]], [I23[1], I13[1]], color='gray', ls=ls_style, lw=1)
                    if I24:
                        self.ax.plot([I12[0], I24[0]], [I12[1], I24[1]], color='gray', ls=ls_style, lw=1)
                        self.ax.plot([I14[0], I24[0]], [I14[1], I24[1]], color='gray', ls=ls_style, lw=1)

                    self.ax.text(limit*0.95, -limit*0.45, ic_coords_txt, fontsize=12, color='purple', ha='right', va='bottom', bbox=dict(facecolor='white', alpha=0.8))

            self.ax.legend(loc='upper right', fontsize=14)
            self.canvas.draw()
        except: pass

    def toggle_mode(self):
        state = tk.NORMAL if self.mode_var.get() == "Slider" else tk.DISABLED
        self.slider.config(state=state); self.angle_entry.config(state=state)

    def on_click(self, event):
        if self.mode_var.get() == "Mouse" and event.button == 1:
            self.is_dragging = True; self.handle_mouse_logic(event)

    def on_release(self, event): self.is_dragging = False

    def on_drag(self, event):
        if self.is_dragging: self.handle_mouse_logic(event)

    def handle_mouse_logic(self, event):
        if event.inaxes == self.ax:
            angle = np.degrees(np.arctan2(event.ydata, event.xdata))
            if self.dead_points: angle = max(self.dead_points[0], min(angle, self.dead_points[1]))
            self.angle_var.set(round(angle, 2)); self.render_plot()

if __name__ == "__main__":
    root = tk.Tk(); root.geometry("1600x1000"); app = RobotEngineeringLinkageApp(root); root.mainloop()