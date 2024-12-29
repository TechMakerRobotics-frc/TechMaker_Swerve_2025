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
        self.field_width_m = 16.54
        self.field_height_m = 8.21

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
        self.canvas.bind("<Button-1>", self.start_circle)
        self.canvas.bind("<B1-Motion>", self.draw_circle)
        self.canvas.bind("<ButtonRelease-1>", self.end_circle)

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

    def start_circle(self, event):
        self.start_x = event.x
        self.start_y = event.y

    def draw_circle(self, event):
        self.canvas.delete("preview")
        radius = ((event.x - self.start_x) ** 2 + (event.y - self.start_y) ** 2) ** 0.5
        self.canvas.create_oval(
            self.start_x - radius, self.start_y - radius,
            self.start_x + radius, self.start_y + radius,
            outline="red", tag="preview"
        )

    def end_circle(self, event):
        if not self.current_zone:
            return

        end_x = event.x
        end_y = event.y

        # Calcular o raio do círculo
        radius_px = ((end_x - self.start_x) ** 2 + (end_y - self.start_y) ** 2) ** 0.5
        radius_m = radius_px * self.scale_x

        # Converter o centro para coordenadas do campo real
        center_x_m = self.start_x * self.scale_x
        center_y_m = (self.image_height_px - self.start_y) * self.scale_y

        self.zones[self.current_zone].append({
            "center_x": round(center_x_m, 2),
            "center_y": round(center_y_m, 2),
            "radius": round(radius_m, 2)
        })

        # Desenhar o círculo final no canvas
        self.canvas.create_oval(
            self.start_x - radius_px, self.start_y - radius_px,
            self.start_x + radius_px, self.start_y + radius_px,
            outline="blue"
        )

    def save_zones(self):
        # Caminho para salvar o arquivo JSON
        save_path = "src/main/java/frc/robot/util/zones/zones.json"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

        # Salvar as zonas
        data_to_save = {"zones": self.zones}
        try:
            with open(save_path, "w") as f:
                json.dump(data_to_save, f, indent=4)
            print(f"Zonas salvas em {save_path}")
        except Exception as e:
            print(f"Erro ao salvar o arquivo: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    editor = ZoneEditor(root, "src/main/java/frc/robot/util/zones/Crescendo.png")
    print(f"Diretório atual: {os.getcwd()}")
    root.mainloop()
