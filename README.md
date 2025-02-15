# Path Planning with A* Algorithm and Potential Fields

This repository contains a Python-based path planning implementation using the **A* algorithm** combined with **potential field modifications** to optimize the pathfinding process. The code processes images of maps, extracts obstacles, generates potential fields, and finds paths using A* with and without potential modifications.

---

##  Features

- **Image Processing:** Converts input map images into binary grids.
- **A* Pathfinding:** Implements A* algorithm for path planning.
- **Potential Fields:** Enhances navigation using Gaussian potential functions.
- **Visualization:** Generates 2D and 3D plots for potential fields and paths.
- **Multiple Scenarios:** Supports multiple planners with different map inputs.
- **Automated Graph Saving:** Saves all visualization outputs as separate images.

---

##  Requirements

Ensure you have the following dependencies installed:

```bash
pip install numpy opencv-python matplotlib
```

---

##  Usage

1. **Prepare Your Maps:** Place your map images (e.g., `map.png`, `map1.png`, `map2.png`) in the project directory.
2. **Run the script:** Execute the Python script to process maps and generate paths.
3. **Check the Outputs:** Results are displayed in graphs and saved in the `separate_graphs_output` directory.

### Running the Script

```bash
python pathplanning.py
```

---

##  Output Structure

After execution, the script generates and saves the following outputs:

```
separate_graphs_output/
│── planner_1_original_paths.png
│── planner_1_potential_2d.png
│── planner_1_potential_3d.png
│── planner_1_path_distances.png
│── ... (Similar files for other planners)
```

Each planner corresponds to a different map input.

---

## Visualizations

The script generates the following graphs:

- **Paths on Original Image:** Compares paths with and without potential field modifications.
- **2D Potential Field:** Shows obstacle effects using Gaussian potential fields.
- **3D Potential Field:** Provides a 3D perspective of the field landscape.(x-cord,y-cord,Potantiel Score)
- **Path Distance Analysis:** Compares path lengths for evaluation.

---

##  Code Structure

- `PathPlanning` class: Handles map processing, potential field generation, and A* pathfinding.
- `process_map()`: Extracts contours and obstacles from the input map.
- `compute_potential_field()`: Generates a potential field using Gaussian functions.
- `find_paths()`: Runs A* algorithm on both standard and modified grids.
- `save_graphs_with_legends()`: Saves all graphs for further analysis.

---

##  References

[1] Albayrak, M., & Sümen, A. M. (2023). A Yıldız ve Akın Algoritmaları ile Otonom Sürü 
Sistemleri için yol Bulma. Bilişim Teknolojileri Dergisi, 16(4), 251-261. 
https://doi.org/10.17671/gazibtd.1236552
[2] Nennioğlu, A. K., & Köroğlu, T. (2018). Otonom Araçlarda Hareket Planlaması. Artıbilim: 
Adana BTÜ Fen Bilimleri Dergisi, 1(2), 23-30. https://dergipark.org.tr/tr/download/article-file/614340
[3] Djojo, A., & Karyono, T. (2018). A*, Floyd-Warshall ve Dijkstra Algoritmalarının 
Karşılaştırılması. Anatolian Journal of Computer Sciences, 3(1), 45-52. 
https://dergipark.org.tr/tr/download/article-file/668518
[4] Yıldız, H., Özer, H. Ö., Durak, B., & Uzal, E. (2024). Radyal Baz Fonksiyonu (RBF) kullanan 
Ağsız (Meshless) Çözüm Yöntemlerinde Şekil Parametresi ve Merkez Nokta Sayısının Çözüme Etkisi. 
Karadeniz Fen Bilimleri Dergisi, 14(3), 1301-1321. https://doi.org/10.31466/kfbd.1455017
[5] Durak, B. (2020). Adi ve Kısmi Diferansiyel Denklemlerin Çözümlerinin Kollokasyon 
Yöntemiyle Bulunması. Gümüşhane Üniversitesi Fen Bilimleri Dergisi, 10(4), 1136-1143. 
https://doi.org/10.17714/gumusfenbil.681276
[6] Altınkaynak, A. (2020). Ağsız Yöntem Uygulamaları için Trigonometri Tabanlı Radyal 
Özelliğe Sahip Yeni Bir Temel Fonksiyon. International Journal of Advances in Engineering and 
Pure Sciences, 32(1), 96-110. https://doi.org/10.7240/jeps.581959
[7] Kabir, R., Watanobe, Y., Islam, M. R., & Naruse, K. (2023). Enhanced Robot Motion Block of 
A-star Algorithm for Robotic Path Planning. arXiv preprint, arXiv:2312.15738. 
https://arxiv.org/abs/2312.15738
[8] Zheng, H., & Mangharam, R. (2023). Differentiable Trajectory Generation for Car-like Robots 
with Interpolating Radial Basis Function Networks. arXiv preprint, arXiv:2303.00981. 
https://arxiv.org/abs/2303.00981
[9] Yonetani, R., Taniai, T., Barekatain, M., Nishimura, M., & Kanezaki, A. (2020). Path Planning 
using Neural A* Search. arXiv preprint, arXiv:2009.07476. https://arxiv.org/abs/2009.07476
[10] Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic 
Determination of Minimum Cost Paths. IEEE Transactions on Systems Science and Cybernetics, 4(2), 
100-107.

---

##Related Publication

This work has been published in the 5. INTERNATIONAL TRAKYA SCIENTIFIC RESEARCH CONFERANCE EDİRNE,Noc,7 2024. You can find the related conference paper at the following link:

[ISARC Conference Paper (Page 93)](https://www.isarconference.org/_files/ugd/6dc816_79b5de171d7c49b5b2723cd7151ffec6.pdf#page=93)

##  Contact

For any questions, please open an issue or reach out via [LinkedIn]((https://www.linkedin.com/in/arslanakin/)).


