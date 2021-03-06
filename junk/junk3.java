package searchclient;

import java.util.Comparator;
import java.util.LinkedList;
import java.util.Vector;
import java.util.HashSet;
import java.util.HashMap;
import java.util.ArrayList;

import java.lang.Math;

import searchclient.NotImplementedException;

// Used ~ using dijstras ~ ADAPTED FROM https://www.geeksforgeeks.org/greedy-algorithms-set-6-dijkstras-shortest-path-algorithm/ for implementation of DIJ

public abstract class Heuristic implements Comparator<Node> {
	/* Private member variables */
	private int distances[][][];
	private ArrayList<int[]> goals;
	private int MR, MC;
	/* Member functions */
	public Heuristic(Node initialState) {
		// Init vals for Max row and max column
		this.MR = initialState.MAX_ROW;
		this.MC = initialState.MAX_COL;
		/****** Preprocessing ******/
		// 1.) Set list of goal states coords and values
		this.goals = new ArrayList<int[]>();
		for (int row = 0; row < initialState.MAX_ROW; row++) {
			for (int col = 0; col < initialState.MAX_COL; col++) {
				if (initialState.goals[row][col] > 0) {
					int[] nP = {row, col, initialState.goals[row][col] - 32};
					this.goals.add(nP);
				}
			}
		}

		// 2.) For each goal, compute dijstras to all other nodes in the graph
		this.distances  = new int [initialState.MAX_ROW][initialState.MAX_COL][this.goals.size()];
		Boolean[][][] sptSet = new Boolean [initialState.MAX_ROW][initialState.MAX_COL][this.goals.size()]; // true if already in the path
		int coords[] = new int [2];
		int r, c;
		// Init all distances as infinite
		for (int goal = 0; goal < this.goals.size() ; goal++) {
			for (int row = 0; row < initialState.MAX_ROW; row++) {
				for (int col = 0; col < initialState.MAX_COL; col++) {
					this.distances[row][col][goal] = Integer.MAX_VALUE;
					sptSet[row][col][goal] = false;
				}
			}
			// Set source node distance to 0
			this.distances[this.goals.get(goal)[0]][this.goals.get(goal)[1]][goal] = 0;
			// Find shortest path to all vertices
			for (int i = 0; i < initialState.MAX_ROW*initialState.MAX_COL; i++) {
				// Find next coords
				coords = this.minDistance(goal, sptSet);
				r = coords[0];
				c = coords[1];
				sptSet[r][c][goal] = true;
				// Check four neighbor cells to update
				if (r > 0 && !initialState.walls[r-1][c] && !sptSet[r-1][c][goal] && this.distances[r][c][goal] != Integer.MAX_VALUE && this.distances[r][c][goal] + 1 < this.distances[r-1][c][goal]) {
					this.distances[r-1][c][goal] = this.distances[r][c][goal] + 1;
				}
				if (r < initialState.MAX_ROW - 1 && !initialState.walls[r+1][c] && !sptSet[r+1][c][goal] && this.distances[r][c][goal] != Integer.MAX_VALUE && this.distances[r][c][goal] + 1 < this.distances[r+1][c][goal]) {
					this.distances[r+1][c][goal] = this.distances[r][c][goal] + 1;
				}
				if (c > 0 && !initialState.walls[r][c-1] && !sptSet[r][c-1][goal] && this.distances[r][c][goal] != Integer.MAX_VALUE && this.distances[r][c][goal] + 1 < this.distances[r][c-1][goal]) {
					this.distances[r][c-1][goal] = this.distances[r][c][goal] + 1;
				}		
				if (c < initialState.MAX_COL - 1 && !initialState.walls[r][c+1] && !sptSet[r][c+1][goal] && this.distances[r][c][goal] != Integer.MAX_VALUE && this.distances[r][c][goal] + 1 < this.distances[r][c+1][goal]) {
					this.distances[r][c+1][goal] = this.distances[r][c][goal] + 1;
				}				
			}
		}
	}
	// Calculate the next minimum node in the coords and return coords
	private int[] minDistance(int goal, Boolean[][][] sptSet) {
		int mini = Integer.MAX_VALUE;
		int[] m_coords = {-1, -1};
		for (int row = 0; row < this.MR; row++) {
			for (int col = 0; col < this.MC; col++) {
				if (!sptSet[row][col][goal] && this.distances[row][col][goal] <= mini) {
					mini = this.distances[row][col][goal];
					m_coords[0] = row;
					m_coords[1] = col;
				}
			}
		}
		return m_coords;
	}
	// Heuristic function here
	public int h(Node n) {
		int h_val = 0;
		int min_dist, min_ind;
		HashMap<Integer, HashSet<Integer>> taken = new HashMap<Integer, HashSet<Integer>>();
		for (int goal = 0; goal < this.goals.size(); goal++) {
			if (!taken.containsKey(this.goals.get(goal)[2])) {
				taken.put(this.goals.get(goal)[2], new HashSet<Integer>());
			}
			min_dist = Integer.MAX_VALUE;
			min_ind = -1;
			for (int row = 0; row < n.MAX_ROW; row++) {
				for (int col = 0; col < n.MAX_COL; col++) {
					if (n.boxes[row][col] == this.goals.get(goal)[2] && this.distances[row][col][goal] < min_dist && taken.get(this.goals.get(goal)[2]).contains(row * this.MC + col)) {
						min_dist = this.distances[row][col][goal];
						min_ind = row * this.MC + col;
					}
				}
			}
			h_val += min_dist;
			taken.get(this.goals.get(goal)[2]).add(min_ind);
		}
		return h_val;
	}

	public abstract int f(Node n);

	@Override
	public int compare(Node n1, Node n2) {
		return this.f(n1) - this.f(n2);
	}

	public static class AStar extends Heuristic {
		public AStar(Node initialState) {
			super(initialState);
		}

		@Override
		public int f(Node n) {
			return n.g() + this.h(n);
		}

		@Override
		public String toString() {
			return "A* evaluation";
		}
	}

	public static class WeightedAStar extends Heuristic {
		private int W;

		public WeightedAStar(Node initialState, int W) {
			super(initialState);
			this.W = W;
		}

		@Override
		public int f(Node n) {
			return n.g() + this.W * this.h(n);
		}

		@Override
		public String toString() {
			return String.format("WA*(%d) evaluation", this.W);
		}
	}

	public static class Greedy extends Heuristic {
		public Greedy(Node initialState) {
			super(initialState);
		}

		@Override
		public int f(Node n) {
			return this.h(n);
		}

		@Override
		public String toString() {
			return "Greedy evaluation";
		}
	}
}