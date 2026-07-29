# Rapidly-Exploring Random Trees (RRT, RRT*, RRT-Scan)

A collection of autonomous sampling-based motion planning algorithms including **RRT**, **Bi-directional RRT**, **Goal-Biased RRT**, **RRT\***, and **KD-Tree Accelerated RRT\*-Scan**.

---

## Implemented Algorithms

- **`rrt_brute_force.py`**: Basic RRT tree expansion in 2D space.
- **`RRT_with_obstacles.py`**: Obstacle-constrained RRT motion planning.
- **`Goal_biased_RRT.py`**: RRT with goal-sampling bias for faster convergence.
- **`RRT_bidirectional.py`**: Bi-directional RRT growing dual trees from start and goal.
- **`RRTStar_Obstacles.py`**: Optimal RRT\* algorithm with path rewiring.
- **`rrt_star_kdtree.py`**: KD-Tree accelerated RRT\* nearest-neighbor query optimization.
- **`RRTstar_Scan.py`**: RRT\*-Scan combining spatial roadmaps and sensor scan sweeps.

---

## Installation & Setup

Clone the repository and install dependencies:

```bash
git clone https://github.com/vampcoder/Rapidly-Exploring-Random-Tree-Star-Scan.git
cd Rapidly-Exploring-Random-Tree-Star-Scan
pip install -r requirements.txt
```

---

## Usage

Run any RRT motion planning script:

```bash
# Goal-Biased RRT
python3 Goal_biased_RRT.py

# KD-Tree Accelerated RRT*
python3 rrt_star_kdtree.py
```

---

## License

MIT License
