import tkinter as tk
from tkinter import scrolledtext, simpledialog, messagebox
from PIL import Image, ImageTk, ImageDraw, ImageFont

class FrameBuffer:
    def __init__(self, canvas, width=160, height=128, scale=5):
        self.canvas = canvas
        self.width = width
        self.height = height
        self.scale = scale
        self.image = Image.new("RGB", (width, height), (0, 0, 0))
        self.grid_image = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        self.draw = ImageDraw.Draw(self.image)
        self.grid_draw = ImageDraw.Draw(self.grid_image)
        self.cursor_x = 0
        self.cursor_y = 0
        self.text_size = 1
        self.text_color = (255, 255, 255)
        self.font = ImageFont.truetype("DejaVuSans.ttf", 8 * self.text_size)
        self.grid_color = (50, 50, 50, 255)
        self.grid_enabled = True
        self.variables = {}
        self.create_grid()

    def create_grid(self):
        for i in range(0, self.width, 10):
            self.grid_draw.line([(i, 0), (i, self.height)], fill=self.grid_color)
            self.grid_draw.text((i, 0), str(i), font=self.font, fill=self.grid_color)
        for j in range(0, self.height, 10):
            self.grid_draw.line([(0, j), (self.width, j)], fill=self.grid_color)
            self.grid_draw.text((0, j), str(j), font=self.font, fill=self.grid_color)

    def rgb565_to_rgb(self, color):
        r = (color >> 11) & 0x1F
        g = (color >> 5) & 0x3F
        b = color & 0x1F
        return (r << 3, g << 2, b << 3)

    def rgb_to_rgb565(self, r, g, b):
        return (r & 0xF8) << 8 | (g & 0xFC) << 3 | (b >> 3)

    def setCursor(self, x, y):
        self.cursor_x = x
        self.cursor_y = y

    def getCursorX(self):
        return self.cursor_x
    
    def getCursorY(self):
        return self.cursor_y

    def printf(self, format_str, *args):
        text = format_str % args
        for char in text:
            if char == '\n':  # 检测到换行字符
                self.cursor_x = 0
                self.cursor_y += 8 * self.text_size
                continue  # 继续下一个字符的处理

            if self.cursor_x >= self.width:
                self.cursor_x = 0
                self.cursor_y += 8 * self.text_size

            if self.cursor_y >= self.height:
                self.cursor_y = 0

            self.draw.text((self.cursor_x, self.cursor_y), char, font=self.font, fill=self.text_color)
            self.cursor_x += 6 * self.text_size

        self.update_display()

    def setTextSize(self, n):
        self.text_size = max(1, n)
        try:
            self.font = ImageFont.truetype("DejaVuSans.ttf", 8 * self.text_size)
        except IOError:
            self.font = ImageFont.load_default()

    def setTextColor(self, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.text_color = rgb_color

    def drawRect(self, x, y, w, h, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.draw.rectangle([x, y, x + w, y + h], outline=rgb_color)
        self.update_display()

    def fillRect(self, x, y, w, h, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.draw.rectangle([x, y, x + w, y + h], fill=rgb_color)
        self.update_display()

    def drawLine(self, x1, y1, x2, y2, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.draw.line([x1, y1, x2, y2], fill=rgb_color)
        self.update_display()

    def drawFastHLine(self, x, y, w, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.draw.line([x, y, x + w, y], fill=rgb_color)
        self.update_display()

    def drawFastVLine(self, x, y, h, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.draw.line([x, y, x, y + h], fill=rgb_color)
        self.update_display()

    def fillScreen(self, color):
        rgb_color = self.rgb565_to_rgb(color)
        self.draw.rectangle([0, 0, self.width, self.height], fill=rgb_color)
        self.update_display()

    def display(self):
        self.update_display()

    def setVar(self, name, value):
        exec(f"global {name}; {name} = {value}", self.variables)
        self.variables.pop('__builtins__', None)

    def getVar(self, name):
        return eval(name, self.variables)

    def update_display(self):
        display_image = self.image.copy()
        if self.grid_enabled:
            display_image.paste(self.grid_image, (0, 0), self.grid_image)
        scaled_image = display_image.resize((self.width * self.scale, self.height * self.scale), Image.NEAREST)
        self.tk_image = ImageTk.PhotoImage(scaled_image)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)

    def enable_grid(self):
        self.grid_enabled = True
        self.update_display()

    def disable_grid(self):
        self.grid_enabled = False
        self.update_display()

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("ST7735 EMU")

        self.canvas = tk.Canvas(root, width=160*5, height=128*5, bg="black")
        self.canvas.grid(row=0, column=0, padx=10, pady=10)

        self.error_text = tk.Text(root, width=62, height=4, fg="red", state=tk.DISABLED)
        self.error_text.grid(row=1, column=0, padx=10, pady=0)

        self.frame_buffer = FrameBuffer(self.canvas)

        self.input_text = scrolledtext.ScrolledText(root, width=58, height=16)
        self.input_text.grid(row=0, column=1, padx=10, pady=10)
        self.input_text.bind("<KeyRelease>", self.on_input_change)

        self.command_text = tk.Entry(root, width=51)
        self.command_text.grid(row=1, column=1, padx=10, pady=0)
        self.command_text.bind("<Return>", self.on_command_enter)

        self.var_manager_button = tk.Button(root, text="Variable Manager", command=self.open_var_manager)
        self.var_manager_button.grid(row=2, column=1, padx=10, pady=10)

    def on_input_change(self, event=None):
        self.run_commands()

    def run_commands(self):
        self.error_text.config(state=tk.NORMAL)
        self.error_text.delete("1.0", tk.END)
        commands = self.input_text.get("1.0", tk.END).strip().split(';')
        for command in commands:
            command = command.split('//')[0].strip()
            if command:
                self.execute_command(command)
        self.error_text.config(state=tk.DISABLED)

    def on_command_enter(self, event=None):
        command = self.command_text.get().strip()
        self.command_text.delete(0, tk.END)
        if command:
            self.execute_tool_command(command)
        else:
            self.error_text.config(state=tk.NORMAL)
            self.error_text.delete("1.0", tk.END)
            self.error_text.insert(tk.END, "No command\n")
            self.error_text.config(state=tk.DISABLED)

    def execute_command(self, command):
        try:
            exec(f"self.frame_buffer.{command}", {'self': self, **self.frame_buffer.variables})
        except Exception as e:
            self.error_text.config(state=tk.NORMAL)
            self.error_text.insert(tk.END, f"Error executing command '{command}': {e}\n")
            self.error_text.config(state=tk.DISABLED)

    def execute_tool_command(self, command):
        self.error_text.config(state=tk.NORMAL)
        self.error_text.delete("1.0", tk.END)
        try:
            parts = command.split()
            if parts[0] == "rgb565" and len(parts) == 4:
                r, g, b = int(parts[1]), int(parts[2]), int(parts[3])
                rgb565 = self.frame_buffer.rgb_to_rgb565(r, g, b)
                self.error_text.insert(tk.END, f"RGB565: {hex(rgb565)}\n")
            elif parts[0] == "disGrid":
                self.frame_buffer.disable_grid()
                self.error_text.insert(tk.END, "Disable Grid\n")
            elif parts[0] == "enbGrid":
                self.frame_buffer.enable_grid()
                self.error_text.insert(tk.END, "Enable Grid\n")
            elif parts[0] == "setVar" and len(parts) == 3:
                name, value = parts[1], parts[2]
                self.frame_buffer.setVar(name, value)
                self.error_text.insert(tk.END, f"Variable '{name}' set to {value}\n")
                self.frame_buffer.update_display()
                
            elif parts[0] == "getVar" and len(parts) == 2:
                name = parts[1]
                value = self.frame_buffer.getVar(name)
                self.error_text.insert(tk.END, f"{name} = {value}\n")
            elif parts[0] == "help":
                self.error_text.insert(tk.END, f"Commands:\n"
                                              f"rgb565 <R> <G> <B>: Convert RGB to RGB565 hex.\n"
                                              f"enbGrid: Enable Grid.\n"
                                              f"disGrid: Disable Grid.\n"
                                              f"setVar <name> <value>: Set a variable.\n"
                                              f"getVar <name>: Get a variable value.\n"
                                              f"help: Show this help message.\n")
            else:
                self.error_text.insert(tk.END, f"Unknown command: {command}. Use 'help' to view help\n")
        except Exception as e:
            self.error_text.insert(tk.END, f"Error executing tool command '{command}': {e}\n")
        self.error_text.config(state=tk.DISABLED)

    def open_var_manager(self):
        VarManager(self.frame_buffer)

class VarManager(tk.Toplevel):
    def __init__(self, frame_buffer):
        super().__init__()
        self.frame_buffer = frame_buffer
        self.title("Variable Manager")
        self.geometry("600x600")

        self.var_listbox = tk.Listbox(self)
        self.var_listbox.pack(fill=tk.BOTH, expand=True)
        self.var_listbox.bind('<Double-1>', self.edit_var)

        self.refresh_button = tk.Button(self, text="Refresh", command=self.refresh)
        self.refresh_button.pack(side=tk.LEFT)

        self.add_button = tk.Button(self, text="Add Variable", command=self.add_var)
        self.add_button.pack(side=tk.LEFT)

        self.delete_button = tk.Button(self, text="Delete Variable", command=self.delete_var)
        self.delete_button.pack(side=tk.LEFT)

        self.refresh()

    def refresh(self):
        self.var_listbox.delete(0, tk.END)
        for name, value in self.frame_buffer.variables.items():
            self.var_listbox.insert(tk.END, f"{name}: {value}")

    def add_var(self):
        name = simpledialog.askstring("Variable Name", "Enter variable name:")
        value = simpledialog.askstring("Variable Value", "Enter variable value:")
        if name and value is not None:
            self.frame_buffer.setVar(name, eval(value))
            self.refresh()

    def delete_var(self):
        selected = self.var_listbox.curselection()
        if selected:
            var_name = self.var_listbox.get(selected[0]).split(":")[0].strip()
            del self.frame_buffer.variables[var_name]
            self.refresh()

    def edit_var(self, event):
        selected = self.var_listbox.curselection()
        if selected:
            var_name = self.var_listbox.get(selected[0]).split(":")[0].strip()
            new_value = simpledialog.askstring("Edit Variable", f"Enter new value for {var_name}:")
            if new_value is not None:
                self.frame_buffer.setVar(var_name, eval(new_value))
                self.refresh()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
