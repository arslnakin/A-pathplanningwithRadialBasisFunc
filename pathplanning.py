import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import heapq
import os


class PathPlanning:
    def __init__(self, image_path, start, end, threshold_value=50, scale_factor=0.58, potential_amplitude=5):
        self.image_path = image_path
        self.start = start
        self.end = end
        self.threshold_value = threshold_value
        self.scale_factor = scale_factor
        self.potential_amplitude = potential_amplitude

        # Görüntü ve ilgili veriler
        self.original_image = None
        self.processed_image = None
        self.contour_only_image = None
        self.contour_data = None

        # Grid ve potansiyel alan
        self.grid = None
        self.potential = None
        self.X = None
        self.Y = None

        # Yol bilgileri
        self.path_with_potential = None
        self.path_without_potential = None
        self.path_distances_potential = None
        self.path_distances_no_potential = None

    @staticmethod
    def convert_image(image_path):
        """Resmi siyah alanlar 1, beyaz alanlar 0 olacak şekilde binary grid'e çevirir."""
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"Görüntü '{image_path}' bulunamadı!")
        _, binary_image = cv2.threshold(image, 127, 1, cv2.THRESH_BINARY_INV)
        return binary_image.astype(np.uint8)

    @staticmethod
    def gauss(x, y, xm, ym, ex, ey, A=10):
        """Gauss fonksiyonu ile potansiyel alan oluşturur."""
        return A * np.exp(-ex**2 * (x - xm)**2 - ey**2 * (y - ym)**2)

    @staticmethod
    def calculate_path_distance(end_point, points):
        """Verilen noktaların end noktasına olan mesafelerini hesaplar."""
        return np.array([np.linalg.norm(np.array(point) - np.array(end_point)) for point in points])

    @staticmethod
    def a_star(grid, start, end):
        """A* algoritması ile yol bulma."""
        def heuristic(a, b):
            return ((a[0] - b[0])**2 + (a[1] - b[1])**2)/(100)

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:
                    if grid[neighbor[1], neighbor[0]] == 1:
                        continue

                    new_cost = cost_so_far[current] + grid[neighbor[1], neighbor[0]]

                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, end)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        return []

    def process_map(self):
        """Görüntüyü işler ve konturları bulur."""
        image = cv2.imread(self.image_path)
        if image is None:
            raise FileNotFoundError(f"Görüntü '{self.image_path}' bulunamadı!")
        self.original_image = image.copy()
        self.contour_only_image = image.copy()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, self.threshold_value, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.contour_data = []

        for contour in contours:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int32(box)

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            distans_list = np.zeros([4, 2])
            for i in range(len(box)):
                p = box[i]
                distans_list[i, :] = [abs(cx - p[0]), abs(cy - p[1])]

            self.contour_data.append({
                "center": (cx, cy),
                "corners": box.tolist(),
                "min": (min(distans_list[:, 0]), min(distans_list[:, 1])),
                "max": (max(distans_list[:, 0]), max(distans_list[:, 1]))
            })

    def compute_potential_field(self):
        """Potansiyel alan hesaplar."""
        self.grid = self.convert_image(self.image_path).astype(np.float64)
        x = np.linspace(0, self.original_image.shape[1], self.original_image.shape[1])
        y = np.linspace(0, self.original_image.shape[0], self.original_image.shape[0])

        xm = np.array([data['center'][0] for data in self.contour_data])
        ym = np.array([data['center'][1] for data in self.contour_data])
        Dx = np.array([data['min'][0] for data in self.contour_data])
        Dy = np.array([data['min'][1] for data in self.contour_data])
        ex = self.scale_factor * 1 / Dx
        ey = self.scale_factor * 1 / Dy

        self.X, self.Y = np.meshgrid(x, y)
        self.potential = np.zeros_like(self.X, dtype=float)
        for i in range(len(xm)):
            self.potential += self.gauss(self.X, self.Y, xm[i], ym[i], ex[i], ey[i], self.potential_amplitude)

    def find_paths(self):
        """A* algoritması ile yolları bulur."""
        self.path_with_potential = self.a_star(self.potential, self.start, self.end)
        self.path_without_potential = self.a_star(self.grid, self.start, self.end)

        self.path_distances_potential = self.calculate_path_distance(self.end, self.path_with_potential)
        self.path_distances_no_potential = self.calculate_path_distance(self.end, self.path_without_potential)


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Tüm planlayıcıları listeye ekleyelim
planners = [
    PathPlanning(image_path="map.png", start=(10, 10), end=(110, 110), scale_factor=0.6, potential_amplitude=5),
    PathPlanning(image_path="map1.png", start=(20, 10), end=(110, 110), scale_factor=0.6, potential_amplitude=5),
    PathPlanning(image_path="map2.png", start=(10, 10), end=(110, 110), scale_factor=0.6, potential_amplitude=5)
]

# Her planlayıcı için işlemleri gerçekleştirelim
for planner in planners:
    planner.process_map()
    planner.compute_potential_field()
    planner.find_paths()


# Sanallaştırma
fig = plt.figure(figsize=(50, len(planners) * 12))  # Grafiği büyüttük

for idx, planner in enumerate(planners):
    start = planner.start
    end = planner.end
    path = planner.path_with_potential
    path1 = planner.path_without_potential
    p1 = planner.path_distances_potential
    p2 = planner.path_distances_no_potential

    # 1. Grafik: Orijinal Görüntü üzerinde yollar
    ax1 = fig.add_subplot(len(planners), 4, idx * 4 + 1)
    ax1.imshow(cv2.cvtColor(planner.original_image, cv2.COLOR_BGR2RGB))
    path_x, path_y = zip(*path)
    path1_x, path1_y = zip(*path1)
    ax1.plot(path1_x, path1_y, color="yellow", linewidth=2, label="Gauss Formu eklenmemiş yol")
    ax1.plot(path_x, path_y, color="green", linewidth=2, label="Potansiyelli Yol")
    ax1.scatter(*start, color="red", label="Başlangıç")
    ax1.scatter(*end, color="blue", label="Bitiş")
    ax1.set_title(f"Planner {idx + 1}: Paths on Original Image", fontsize=14)
    ax1.axis("off")
    ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10, markerscale=0.8)

    # 2. Grafik: Potansiyel Alan (2D)
    ax2 = fig.add_subplot(len(planners), 4, idx * 4 + 2)
    ax2.imshow(planner.potential, extent=(0, planner.original_image.shape[1], planner.original_image.shape[0], 0), cmap='hot', interpolation='nearest')
    ax2.scatter(*start, color="red", label="Başlangıç")
    ax2.scatter(*end, color="blue", label="Bitiş")
    ax2.plot(path_x, path_y, color="green", linewidth=2, label="Potansiyelli Yol")
    ax2.plot(path1_x, path1_y, color="yellow", linewidth=2, label="a* Yol")
    ax2.set_title(f"Planner {idx + 1}: Potential Field (2D)", fontsize=14)
    ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10, markerscale=0.8)

    # 3. Grafik: Potansiyel Alan (3D)
    ax3 = fig.add_subplot(len(planners), 4, idx * 4 + 3, projection='3d')
    ax3.plot_surface(planner.X, planner.Y, planner.potential, cmap='viridis', edgecolor='none', alpha=0.8)

    # A* Yollarını 3D olarak göster
    path_z = [planner.potential[int(py), int(px)] for px, py in path]
    path1_z = [planner.potential[int(py), int(px)] for px, py in path1]
    ax3.plot(path_x, path_y, path_z, color='blue', linewidth=2, label='Potansiyelli Yol')
    ax3.plot(path1_x, path1_y, path1_z, color='yellow', linewidth=2, label='Gauss Formu eklenmemiş yol')
    ax3.scatter(*start, planner.potential[start[1], start[0]], color="red", s=100, label="Başlangıç")
    ax3.scatter(*end, planner.potential[end[1], end[0]], color="blue", s=100, label="Bitiş")
    ax3.set_title(f"Planner {idx + 1}: Potential Field (3D)", fontsize=14)
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Potential')
    ax3.legend(loc='center left', bbox_to_anchor=(1.2, 0.5), fontsize=10, markerscale=0.8)

    # 4. Grafik: Path Distance
    ax4 = fig.add_subplot(len(planners), 4, idx * 4 + 4)
    ax4.plot(range(len(p1)), p1, marker='o', color='green', label='Potansiyelli Yol')
    ax4.plot(range(len(p2)), p2, marker='x', color='yellow', label='Gauss Formu eklenmemiş yol')
    ax4.set_title(f"Planner {idx + 1}: Path Distances", fontsize=14)
    ax4.set_xlabel("Adım", fontsize=12)
    ax4.set_ylabel("Mesafe", fontsize=12)
    ax4.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10, markerscale=0.8)
    ax4.grid(alpha=0.7)

# Grafikler arasındaki boşluğu ayarla
plt.subplots_adjust(right=0.88)  # Sağ tarafta daha fazla yer bırak
plt.tight_layout()
plt.show()
# Çıktı klasörü oluştur
output_dir = "separate_graphs_output"
os.makedirs(output_dir, exist_ok=True)

# Her bir grafiği ayrı ayrı kaydetme
def save_graphs_with_legends(planners, output_dir):
    for idx, planner in enumerate(planners):
        start = planner.start
        end = planner.end
        path = planner.path_with_potential
        path1 = planner.path_without_potential
        p1 = planner.path_distances_potential
        p2 = planner.path_distances_no_potential

        # 1. Grafik: Orijinal Görüntü üzerinde yollar
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(cv2.cvtColor(planner.original_image, cv2.COLOR_BGR2RGB))
        path_x, path_y = zip(*path)
        path1_x, path1_y = zip(*path1)
        ax.plot(path1_x, path1_y, color="yellow", linewidth=2, label="Gauss Formu eklenmemiş yol")
        ax.plot(path_x, path_y, color="green", linewidth=2, label="Potansiyelli Yol")
        ax.scatter(*start, color="red", label="Başlangıç")
        ax.scatter(*end, color="blue", label="Bitiş")
        ax.set_title(f"Planner {idx + 1}: Paths on Original Image", fontsize=14)
        ax.axis("off")
        ax.legend(loc='upper right', fontsize=10)
        plt.savefig(os.path.join(output_dir, f"planner_{idx + 1}_original_paths.png"))
        plt.close(fig)

        # 2. Grafik: Potansiyel Alan (2D)
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(planner.potential, extent=(0, planner.original_image.shape[1], planner.original_image.shape[0], 0), cmap='hot', interpolation='nearest')
        ax.scatter(*start, color="red", label="Başlangıç")
        ax.scatter(*end, color="blue", label="Bitiş")
        ax.plot(path_x, path_y, color="green", linewidth=2, label="Potansiyelli Yol")
        ax.plot(path1_x, path1_y, color="yellow", linewidth=2, label="a* Yol")
        ax.set_title(f"Planner {idx + 1}: Potential Field (2D)", fontsize=14)
        ax.legend(loc='upper right', fontsize=10)
        plt.savefig(os.path.join(output_dir, f"planner_{idx + 1}_potential_2d.png"))
        plt.close(fig)

         # 2. Grafik: Potansiyel Alan (2D)
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(planner.potential, extent=(0, planner.original_image.shape[1], planner.original_image.shape[0], 0), cmap='hot', interpolation='nearest')
        ax.scatter(*start, color="red", label="Başlangıç")
        ax.scatter(*end, color="blue", label="Bitiş")
        ax.set_title(f"Planner {idx + 1}: Potential Field (2D)", fontsize=14)
        ax.legend(loc='upper right', fontsize=10)
        plt.savefig(os.path.join(output_dir, f"planner_{idx + 1}_potential_2dd.png"))
        plt.close(fig)

        # 3. Grafik: Potansiyel Alan (3D)
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(planner.X, planner.Y, planner.potential, cmap='viridis', edgecolor='none', alpha=0.8)
        path_z = [planner.potential[int(py), int(px)] for px, py in path]
        path1_z = [planner.potential[int(py), int(px)] for px, py in path1]
        ax.plot(path_x, path_y, path_z, color='blue', linewidth=2, label='Potansiyelli Yol')
        ax.plot(path1_x, path1_y, path1_z, color='yellow', linewidth=2, label='Gauss Formu eklenmemiş yol')
        ax.scatter(*start, planner.potential[start[1], start[0]], color="red", s=100, label="Başlangıç")
        ax.scatter(*end, planner.potential[end[1], end[0]], color="blue", s=100, label="Bitiş")
        ax.set_title(f"Planner {idx + 1}: Potential Field (3D)", fontsize=14)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Potential')
        ax.legend(loc='upper right', fontsize=10)
        plt.savefig(os.path.join(output_dir, f"planner_{idx + 1}_potential_3d.png"))
        plt.close(fig)

        # 3. Grafik: Potansiyel Alan (3D)
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(planner.X, planner.Y, planner.potential, cmap='viridis', edgecolor='none', alpha=0.8)
        path_z = [planner.potential[int(py), int(px)] for px, py in path]
        path1_z = [planner.potential[int(py), int(px)] for px, py in path1]
        ax.scatter(*start, planner.potential[start[1], start[0]], color="red", s=100, label="Başlangıç")
        ax.scatter(*end, planner.potential[end[1], end[0]], color="blue", s=100, label="Bitiş")
        ax.set_title(f"Planner {idx + 1}: Potential Field (3D)", fontsize=14)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Potential')
        ax.legend(loc='upper right', fontsize=10)
        plt.savefig(os.path.join(output_dir, f"planner_{idx + 1}_potential_3dd.png"))
        plt.close(fig)

        # 4. Grafik: Path Distance
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(range(len(p1)), p1, marker='o', color='green', label='Potansiyelli Yol')
        ax.plot(range(len(p2)), p2, marker='x', color='yellow', label='Gauss Formu eklenmemiş yol')
        ax.set_title(f"Planner {idx + 1}: Path Distances", fontsize=14)
        ax.set_xlabel("Adım", fontsize=12)
        ax.set_ylabel("Mesafe", fontsize=12)
        ax.grid(alpha=0.7)
        ax.legend(loc='upper right', fontsize=10)
        plt.savefig(os.path.join(output_dir, f"planner_{idx + 1}_path_distances.png"))
        plt.close(fig)

    print(f"Tüm grafikler '{output_dir}' klasörüne kaydedildi.")

# Kullanım
save_graphs_with_legends(planners, output_dir)
