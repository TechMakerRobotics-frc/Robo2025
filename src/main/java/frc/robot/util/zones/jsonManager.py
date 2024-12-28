import tkinter as tk
import json
import os
from PIL import Image, ImageTk

class ZoneEditor:
    def __init__(self, root, image_path):
        self.root = root
        self.image_path = image_path
        self.zones = {}
        self.current_zone = None
        self.start_x = self.start_y = None

        # Dimensões oficiais do campo em metros
        self.field_width_m = 16.55
        self.field_height_m = 8.2

        # Configurar a interface gráfica
        self.canvas = tk.Canvas(root)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Carregar a imagem do campo e redimensionar
        self.original_image = Image.open(self.image_path)
        self.image_width_px, self.image_height_px = self.original_image.size
        self.image = ImageTk.PhotoImage(self.resize_image())
        self.image_width_px = self.image.width()
        self.image_height_px = self.image.height()

        # Recalcular a escala automaticamente
        self.scale_x = self.field_width_m / self.image_width_px
        self.scale_y = self.field_height_m / self.image_height_px

        self.canvas.config(width=self.image_width_px, height=self.image_height_px)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image)
        self.canvas.bind("<Button-1>", self.start_rectangle)
        self.canvas.bind("<B1-Motion>", self.draw_rectangle)
        self.canvas.bind("<ButtonRelease-1>", self.end_rectangle)

        self.zone_entry = tk.Entry(root)
        self.zone_entry.pack()
        self.add_zone_button = tk.Button(root, text="Adicionar Zona", command=self.add_zone)
        self.add_zone_button.pack()
        self.save_button = tk.Button(root, text="Salvar Zonas", command=self.save_zones)
        self.save_button.pack()

    def resize_image(self):
        max_width, max_height = self.root.winfo_screenwidth() - 100, self.root.winfo_screenheight() - 100
        aspect_ratio = self.image_width_px / self.image_height_px
        if self.image_width_px > max_width:
            self.image_width_px = max_width
            self.image_height_px = int(self.image_width_px / aspect_ratio)
        if self.image_height_px > max_height:
            self.image_height_px = max_height
            self.image_width_px = int(self.image_height_px * aspect_ratio)
        return self.original_image.resize((self.image_width_px, self.image_height_px), Image.Resampling.LANCZOS)

    def add_zone(self):
        zone_name = self.zone_entry.get()
        if zone_name and zone_name not in self.zones:
            self.zones[zone_name] = []
            self.current_zone = zone_name
            self.zone_entry.delete(0, tk.END)

    def start_rectangle(self, event):
        self.start_x = event.x
        self.start_y = event.y

    def draw_rectangle(self, event):
        self.canvas.delete("preview")
        self.canvas.create_rectangle(self.start_x, self.start_y, event.x, event.y, outline="red", tag="preview")

    def end_rectangle(self, event):
        if not self.current_zone:
            return

        end_x = event.x
        end_y = event.y

        # Definir os limites do retângulo
        x1, y1 = min(self.start_x, end_x), min(self.start_y, end_y)
        x2, y2 = max(self.start_x, end_x), max(self.start_y, end_y)

        # Converter para coordenadas do campo real
        x1_field, y1_field = x1 * self.scale_x, (self.image_height_px - y1) * self.scale_y
        x2_field, y2_field = x2 * self.scale_x, (self.image_height_px - y2) * self.scale_y

        # Gerar uma grade de poses dentro do retângulo
        grid_spacing = 0.1  # Espaçamento da grade em metros
        for x in self.frange(x1_field, x2_field, grid_spacing):
            for y in self.frange(y1_field, y2_field, grid_spacing):
                self.zones[self.current_zone].append({"x": round(x, 2), "y": round(y, 2), "heading": 0.0})
                # Desenhar os pontos no canvas
                canvas_x = x / self.scale_x
                canvas_y = self.image_height_px - (y / self.scale_y)
                self.canvas.create_oval(
                    canvas_x - 2, canvas_y - 2,
                    canvas_x + 2, canvas_y + 2,
                    fill="red"
                )

    def frange(self, start, stop, step):
        while start <= stop:
            yield start
            start += step

    def save_zones(self):
        # Caminho para salvar o arquivo JSON
        save_path = "src/main/java/frc/robot/util/zones/zones.json"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        # Salvar as zonas
        data_to_save = {"zones": self.zones}
        try:
            with open(save_path, "w") as f:
                json.dump(data_to_save, f, indent=4)  # Salvar no formato correto
            print(f"Zonas salvas em {save_path}")
        except Exception as e:
            print(f"Erro ao salvar o arquivo: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    editor = ZoneEditor(root, "src/main/java/frc/robot/util/zones/Crescendo.png")
    print(f"Diretório atual: {os.getcwd()}")
    root.mainloop()
